"""
diag_ramp.py — Ramp ONE motor slowly and log everything to find the trip cause.

Usage:
    python diag_ramp.py /dev/ttyUSB0        # ramp the right motor (default)
    python diag_ramp.py /dev/ttyUSB1 800    # ramp left motor up to command 800

Logs per step: applied command (M), power (P), motor amps (A), battery amps (BA),
voltages (internal/battery), and fault/status flags.  Stops immediately if a
fault appears or the driver stops replying (shutdown).
"""

import sys
import time
from keya_motor import KeyaMotorController, KeyaError

PORT     = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
MAX_CMD  = int(sys.argv[2]) if len(sys.argv) > 2 else 700
STEP     = 25      # command increment per step
DWELL    = 0.4     # seconds held at each step before reading
SETTLE   = 0.15    # delay after commanding before reading telemetry


def q(fn):
    """Query helper: return value or None on failure (driver silent / noise)."""
    try:
        return fn()
    except Exception:
        return None


def main():
    print(f"Ramp diagnostic on {PORT}  (0 -> {MAX_CMD}, step {STEP})")
    print("=" * 78)
    m = KeyaMotorController(PORT, timeout=0.2)
    m.connect()

    # baseline
    for _ in range(3):
        try:
            m.release_emergency_stop()
            break
        except KeyaError:
            time.sleep(0.05)

    v = q(m.read_voltages)
    if v:
        print(f"Baseline: battery={v.battery_v:.1f}V  internal={v.internal_v:.1f}V")
    print(f"{'cmd':>5} {'M':>6} {'P':>6} {'Amotor':>7} {'Abatt':>7} "
          f"{'Vbatt':>6} {'faults':>10} {'status'}")
    print("-" * 78)

    cmd = 0
    tripped = False
    try:
        while cmd <= MAX_CMD:
            try:
                m.set_speed(cmd, verify=False)
            except KeyaError:
                pass
            time.sleep(SETTLE)

            mc  = q(m.read_motor_command)
            p   = q(m.read_motor_power)
            a   = q(m.read_motor_amps)
            ba  = q(m.read_battery_amps)
            vv  = q(m.read_voltages)
            ff  = q(m.read_fault_flags)
            fs  = q(m.read_status_flags)

            # detect shutdown: several reads come back empty
            if mc is None and p is None and a is None:
                print(f"{cmd:>5}  *** DRIVER NOT RESPONDING (shutdown) ***")
                tripped = True
                break

            faults = "ok"
            if ff is not None and ff.any_fault():
                fl = []
                if ff.overheat: fl.append("OVERHEAT")
                if ff.overvoltage: fl.append("OVERVOLT")
                if ff.undervoltage: fl.append("UNDERVOLT")
                if ff.short_circuit: fl.append("SHORT")
                if ff.emergency_stop: fl.append("ESTOP")
                if ff.sepex_excitation_fault: fl.append("SEPEX")
                if ff.mosfet_failure: fl.append("MOSFET")
                if ff.startup_config_fault: fl.append("STARTUP")
                faults = ",".join(fl)

            status = ""
            if fs is not None:
                st = []
                if fs.power_stage_off: st.append("PWR-OFF")
                if fs.stall_detected: st.append("STALL")
                if fs.at_limit: st.append("AT-LIMIT")
                status = ",".join(st) if st else "run"

            astr  = f"{a:7.1f}" if a is not None else "    ?? "
            bastr = f"{ba:7.1f}" if ba is not None else "    ?? "
            vstr  = f"{vv.battery_v:6.1f}" if vv is not None else "    ??"
            mstr  = f"{mc:6d}" if mc is not None else "    ??"
            pstr  = f"{p:6d}" if p is not None else "    ??"
            print(f"{cmd:>5} {mstr} {pstr} {astr} {bastr} {vstr} "
                  f"{faults:>10} {status}")

            if ff is not None and ff.any_fault():
                print(f"      *** FAULT at command {cmd}: {faults} ***")
                tripped = True
                break

            time.sleep(DWELL - SETTLE)
            cmd += STEP

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        for _ in range(5):
            try:
                m.set_speed(0, verify=False)
            except Exception:
                pass
            time.sleep(0.02)
        m.disconnect()

    print("-" * 78)
    if not tripped:
        print(f"Reached command {MAX_CMD} with no fault or shutdown.")
    print("Done.")


if __name__ == '__main__':
    main()
