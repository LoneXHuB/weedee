"""
test_controller.py — Drive the rover with a Logitech Driving Force GT wheel (USB).

Steering scheme: ARCADE MIXING (a wheel can't do pure tank).
    Gas pedal    -> forward speed
    Brake pedal  -> reverse speed
    Steering wheel -> turn (speed difference between left & right motors)
    Turning while stopped = spin in place.

    left_motor  = speed + turn
    right_motor = speed - turn

Wiring (rover's perspective):
    RIGHT motor -> /dev/ttyUSB0
    LEFT  motor -> /dev/ttyUSB1

Requirements (install in the env you run from):
    pip install pyserial pygame

Run:
    python test_controller.py              # drive
    python test_controller.py calibrate    # print live axis/button values
"""

import os
import sys
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")   # no window needed

import pygame
from keya_motor import KeyaMotorController, KeyaError

# ─────────────────────────────────────────────────────────────────────────────
# TUNING — adjust these freely
# ─────────────────────────────────────────────────────────────────────────────

RIGHT_PORT = '/dev/ttyUSB0'   # right motor (rover's perspective)
LEFT_PORT  = '/dev/ttyUSB1'   # left  motor

MAX_SPEED  = 600      # forward/reverse speed cap (1..1000). Start low, raise when tuned.
TURN_MAX   = 250      # max turn authority added/subtracted per side (0..1000)
MIN_OUTPUT = 60       # commands below this stall the motor against the brake —
                      # anything weaker is forced to 0. Keep >= the speed needed
                      # to actually start the wheels moving.
OVERCURRENT_A = 35    # if a motor draws more than this (A), cut output instantly
                      # (fail-safe for a stall, e.g. brake engaged). 0 = disable.
ACCEL      = 2000     # controller accel/decel ramp (0.1 RPM/s units). The factory
                      # default 15000 causes a current inrush that collapses the
                      # battery voltage and trips an undervoltage shutdown. Keep
                      # this gentle. Set on both controllers at startup each run.
SLEW_RATE  = 800      # max commanded speed change per second (ramp). 0 = instant.
STEER_DEADZONE = 0.10 # ignore wheel movement within this of centre

# Flip these if a motor or the steering goes the wrong way.
INVERT_LEFT  = False
INVERT_RIGHT = False
INVERT_STEER = False

# Pick the wheel by name (substring, case-insensitive). Falls back to first joystick.
DEVICE_NAME_HINT = "driving force"

# --- Axis mapping (confirmed for Driving Force GT) ---
STEER_AXIS = 0        # steering wheel: -1 = full left, +1 = full right

GAS_AXIS   = 1        # accelerator pedal  (released=+1.00, pressed=-1.00)
GAS_IDLE   = 1.0
GAS_FULL   = -1.0

BRAKE_AXIS = 2        # brake pedal        (released=+1.00, pressed=-1.00)
BRAKE_IDLE = 1.0
BRAKE_FULL = -1.0

ESTOP_BUTTON = 19     # hold this button to cut motors

UPDATE_HZ    = 30     # control loop rate (lower = less serial traffic/noise)
KEEPALIVE_MS = 250    # resend the current speed at least this often
SERIAL_TIMEOUT = 0.08 # per-read serial timeout (s) — short so noise can't stall
TELEM_HZ     = 4      # how often to read amps / power / faults for the display

# ─────────────────────────────────────────────────────────────────────────────


def pick_joystick():
    """Return an initialised pygame joystick, preferring the wheel by name."""
    pygame.init()
    pygame.joystick.init()
    n = pygame.joystick.get_count()
    if n == 0:
        print("No joystick detected by pygame.", file=sys.stderr)
        sys.exit(1)

    chosen = 0
    for i in range(n):
        j = pygame.joystick.Joystick(i)
        j.init()
        if DEVICE_NAME_HINT.lower() in j.get_name().lower():
            chosen = i
            break

    pad = pygame.joystick.Joystick(chosen)
    pad.init()
    return pad


def get_axis(pad, index: int, default: float = 0.0) -> float:
    """Read an axis safely; return default if the index is out of range."""
    if 0 <= index < pad.get_numaxes():
        return pad.get_axis(index)
    return default


def calibrate(pad) -> None:
    """Print every axis and pressed button live so you can find the mapping."""
    print(f"\nCALIBRATION — device: {pad.get_name()}")
    print(f"  {pad.get_numaxes()} axes, {pad.get_numbuttons()} buttons\n")
    print("Move the wheel and press each pedal/button. Ctrl-C to quit.\n")
    try:
        while True:
            pygame.event.pump()
            axes = [f"a{i}={pad.get_axis(i):+.2f}" for i in range(pad.get_numaxes())]
            pressed = [str(i) for i in range(pad.get_numbuttons()) if pad.get_button(i)]
            btn = f"  buttons:{','.join(pressed)}" if pressed else ""
            print("\r" + "  ".join(axes) + btn + " " * 8, end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nDone.")


def pedal_norm(raw: float, idle: float, full: float) -> float:
    """Normalise a pedal axis to 0.0 (released) .. 1.0 (fully pressed)."""
    span = full - idle
    if span == 0:
        return 0.0
    v = (raw - idle) / span
    return max(0.0, min(1.0, v))


def steer_norm(raw: float) -> float:
    """Wheel axis -> -1..1 with a rescaled deadzone (0 just past the deadzone)."""
    if INVERT_STEER:
        raw = -raw
    if abs(raw) < STEER_DEADZONE:
        return 0.0
    sign = 1.0 if raw > 0 else -1.0
    return sign * (abs(raw) - STEER_DEADZONE) / (1.0 - STEER_DEADZONE)


def slew(current: int, target: int, dt: float) -> int:
    if SLEW_RATE <= 0:
        return target
    max_step = SLEW_RATE * dt
    delta = target - current
    if abs(delta) <= max_step:
        return target
    return int(current + max_step * (1 if delta > 0 else -1))


def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


def deadband_out(v: int) -> int:
    """Force stall-level commands to 0 so the motor never fights the brake weakly."""
    return 0 if abs(v) < MIN_OUTPUT else v


def safe_read(fn, tries: int = 3):
    """Run a telemetry query with retries; return None if all attempts fail."""
    for _ in range(tries):
        try:
            return fn()
        except Exception:
            pass
    return None


def fault_names_set(ff) -> set:
    """Set of active fault names from a FaultFlags object."""
    names = set()
    if ff.overheat:               names.add("OVERHEAT")
    if ff.overvoltage:            names.add("OVERVOLT")
    if ff.undervoltage:           names.add("UNDERVOLT")
    if ff.short_circuit:          names.add("SHORT")
    if ff.emergency_stop:         names.add("ESTOP")
    if ff.sepex_excitation_fault: names.add("SEPEX")
    if ff.mosfet_failure:         names.add("MOSFET")
    if ff.startup_config_fault:   names.add("STARTUP")
    return names


def debounce_fault(ff, prev: set):
    """
    Turn a raw fault read into trustworthy display text.

    Rejects noise: an impossible OVERVOLT+UNDERVOLT combo is discarded, and a
    fault is only reported once it appears on two consecutive reads.
    Returns (display_text, new_prev_set).
    """
    if ff is None:
        return "?", prev                       # unreliable / no reply
    names = fault_names_set(ff)
    if {"OVERVOLT", "UNDERVOLT"} <= names:
        return "noisy?", prev                  # contradictory = corrupted read
    confirmed = names & prev                   # must persist across two reads
    text = ",".join(sorted(confirmed)) if confirmed else "ok"
    return text, names


def status_field(side: str, out: int, amps, pwr, fault_text: str) -> str:
    """Format one motor's status for the live display."""
    a = f"{amps:5.1f}A" if amps is not None else "  ??A"
    p = f"{pwr:+5d}"    if pwr  is not None else "  ?? "
    return f"{side} cmd={out:>+5d} {a} pwr={p} [{fault_text}]"


def main() -> None:
    pad = pick_joystick()
    print(f"Controller: {pad.get_name()}  "
          f"({pad.get_numaxes()} axes, {pad.get_numbuttons()} buttons)")

    if len(sys.argv) > 1 and sys.argv[1].lower().startswith("cal"):
        calibrate(pad)
        return

    print(f"Right motor -> {RIGHT_PORT}")
    print(f"Left  motor -> {LEFT_PORT}")
    right = KeyaMotorController(RIGHT_PORT, timeout=SERIAL_TIMEOUT)
    left  = KeyaMotorController(LEFT_PORT,  timeout=SERIAL_TIMEOUT)
    right.connect()
    left.connect()
    # release_emergency_stop verifies its ack, so retry briefly through any noise.
    for m in (right, left):
        for _ in range(3):
            try:
                m.release_emergency_stop()
                break
            except KeyaError:
                time.sleep(0.05)

    # Soften acceleration on both controllers. The factory default (15000) causes
    # a current inrush that collapses the battery and trips an undervoltage
    # shutdown; reverts on power-cycle, so set it every run.
    for m in (right, left):
        for _ in range(3):
            try:
                m.set_acceleration(ACCEL)
                m.set_deceleration(ACCEL)
                break
            except KeyaError:
                time.sleep(0.05)
    print(f"Acceleration set to {ACCEL} (gentle) on both controllers.")

    # Health check: make sure each controller actually answers before driving.
    for name, m in (("RIGHT", right), ("LEFT", left)):
        v = None
        for _ in range(5):
            try:
                v = m.read_voltages()
                break
            except Exception:
                time.sleep(0.05)
        if v is None:
            print(f"  !! WARNING: {name} controller ({m.port}) is NOT responding.")
            print(f"     Check its power LED, power-cycle it, and check the serial")
            print(f"     wires (TX/RX/GND) to the adapter. Driving anyway.")
        else:
            print(f"  {name} controller OK — battery {v.battery_v:.1f} V")
    time.sleep(0.2)

    print("\nArcade drive active.  Gas=forward, Brake=reverse, Wheel=turn.")
    print(f"MAX_SPEED={MAX_SPEED}  TURN_MAX={TURN_MAX}  MIN_OUTPUT={MIN_OUTPUT}  "
          f"(hold button {ESTOP_BUTTON} for e-stop, Ctrl-C to quit)\n")

    def send(motor, speed):
        """Fire-and-forget speed command; never raises on serial noise."""
        try:
            motor.set_speed(speed, verify=False)
        except KeyaError:
            pass

    cur_left = 0
    cur_right = 0
    sent_left = sent_right = None
    period = 1.0 / UPDATE_HZ
    keepalive = KEEPALIVE_MS / 1000.0
    telem_period = 1.0 / TELEM_HZ
    last = time.monotonic()
    last_send = 0.0
    last_telem = 0.0
    overcurrent = False   # latched stall/overcurrent cutout
    # cached telemetry: amps, power for each motor
    l_amps = r_amps = l_pwr = r_pwr = None
    # fault debounce state + display text
    l_fault_prev, r_fault_prev = set(), set()
    l_fault_txt, r_fault_txt = "ok", "ok"

    try:
        while True:
            pygame.event.pump()
            now = time.monotonic()
            dt = now - last
            last = now

            estop = (pad.get_button(ESTOP_BUTTON)
                     if ESTOP_BUTTON < pad.get_numbuttons() else 0)

            # Read pedals every loop (needed to re-arm after an overcurrent cutout).
            gas   = pedal_norm(get_axis(pad, GAS_AXIS,   GAS_IDLE),   GAS_IDLE,   GAS_FULL)
            brake = pedal_norm(get_axis(pad, BRAKE_AXIS, BRAKE_IDLE), BRAKE_IDLE, BRAKE_FULL)

            # Clear the overcurrent latch only once both pedals are released.
            if overcurrent and gas < 0.05 and brake < 0.05:
                overcurrent = False

            if estop or overcurrent:
                target_left = target_right = 0
            else:
                speed = (gas - brake) * MAX_SPEED        # forward minus reverse
                turn = steer_norm(get_axis(pad, STEER_AXIS)) * TURN_MAX

                tl = int(speed + turn)
                tr = int(speed - turn)
                if INVERT_LEFT:
                    tl = -tl
                if INVERT_RIGHT:
                    tr = -tr
                target_left  = deadband_out(clamp(tl, -MAX_SPEED, MAX_SPEED))
                target_right = deadband_out(clamp(tr, -MAX_SPEED, MAX_SPEED))

            # Ramp the raw accumulator (never deadbanded, or it would pin at 0).
            cur_left  = slew(cur_left,  target_left,  dt)
            cur_right = slew(cur_right, target_right, dt)

            # Deadband only the value actually sent, so stall-level power -> 0.
            out_left  = deadband_out(cur_left)
            out_right = deadband_out(cur_right)

            # Only hit the serial bus when something changed or keepalive expires.
            if (out_left != sent_left or out_right != sent_right
                    or now - last_send >= keepalive):
                send(left,  out_left)
                send(right, out_right)
                sent_left, sent_right = out_left, out_right
                last_send = now

            # Periodically read telemetry (amps / power / faults) for the display.
            if now - last_telem >= telem_period:
                l_amps = safe_read(left.read_motor_amps)
                l_pwr  = safe_read(left.read_motor_power)
                l_fault_txt, l_fault_prev = debounce_fault(
                    safe_read(left.read_fault_flags), l_fault_prev)
                r_amps = safe_read(right.read_motor_amps)
                r_pwr  = safe_read(right.read_motor_power)
                r_fault_txt, r_fault_prev = debounce_fault(
                    safe_read(right.read_fault_flags), r_fault_prev)
                last_telem = now

                # Fail-safe: trip the latch if either motor is over the limit.
                if OVERCURRENT_A > 0:
                    for amps in (l_amps, r_amps):
                        if amps is not None and abs(amps) >= OVERCURRENT_A:
                            overcurrent = True

            if estop:
                tag = "  *** E-STOP ***"
            elif overcurrent:
                tag = "  *** OVERCURRENT — release pedals to reset ***"
            else:
                tag = ""
            print("\r" + status_field("L", out_left,  l_amps, l_pwr, l_fault_txt)
                  + "   " + status_field("R", out_right, r_amps, r_pwr, r_fault_txt)
                  + tag + "   ", end="", flush=True)

            time.sleep(period)

    except KeyboardInterrupt:
        print("\nQuitting.")
    finally:
        # Reliable stop: command zero a few times through any noise, then disconnect.
        for _ in range(5):
            send(left, 0)
            send(right, 0)
            time.sleep(0.02)
        for m in (left, right):
            try:
                m.disconnect()
            except Exception:
                pass
        pygame.quit()


if __name__ == '__main__':
    main()
