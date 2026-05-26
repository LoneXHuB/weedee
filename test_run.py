"""Simple motor test — releases brake via digital output then runs at low speed."""

import time
from keya_motor import KeyaMotorController, KeyaError

PORT = '/dev/ttyUSB0'
SPEED = 200        # -1000 to +1000
DURATION = 5.0     # seconds
with KeyaMotorController(PORT, debug=False) as motor:
    motor.release_emergency_stop()
    time.sleep(0.2)

    print(f"Running at speed {SPEED:+d} for {DURATION:.0f} s  (Ctrl-C to stop early)")
    print(f"{'M':>8}  {'P':>8}  {'amps':>8}")
    print("-" * 32)

    deadline = time.monotonic() + DURATION
    while time.monotonic() < deadline:
        try:
            motor.set_speed(SPEED)
        except KeyaError:
            print("  [controller not responding — trying to recover]")
            time.sleep(0.5)
            try:
                motor.release_emergency_stop()
                time.sleep(0.2)
            except KeyaError:
                pass
            continue

        time.sleep(0.05)

        try:
            m = motor.read_motor_command()
            p = motor.read_motor_power()
            a = motor.read_motor_amps()
            print(f"{m:>+8d}  {p:>+8d}  {a:>7.1f} A")
        except KeyaError:
            print("  [read failed]")

        time.sleep(0.15)

    motor.stop()
    print("-" * 32)
    print("Stopped.")
