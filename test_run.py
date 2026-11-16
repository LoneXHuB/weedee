"""Simple motor test — releases e-stop then runs at low speed.

Usage: python3 test_run.py [port]   (default /dev/ttyUSB0; use 'can0' for CAN bus)
"""

import sys
import time
from keya_motor import KeyaMotorController, KeyaError, KeyaUnsupportedError

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
SPEED = 200        # -1000 to +1000
DURATION = 5.0     # seconds
with KeyaMotorController(PORT, debug=False) as motor:
    print(f"Protocol: {motor.protocol}")
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
            try:
                p = motor.read_motor_power()
            except KeyaUnsupportedError:    # not available on every protocol
                p = 0
            a = motor.read_motor_amps()
            print(f"{m:>+8d}  {p:>+8d}  {a:>7.1f} A")
        except KeyaError:
            print("  [read failed]")

        time.sleep(0.15)

    motor.stop()
    print("-" * 32)
    print("Stopped.")
