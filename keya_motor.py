"""
keya_motor.py — Python library for Keya motor controllers via RS232.

Serial settings: 115200 baud, 8-N-1.
Commands are ASCII, terminated with carriage return (\\r).
Acknowledgment: '+' for write commands, response string for queries, '-' on error.

Typical usage:
    from keya_motor import KeyaMotorController

    with KeyaMotorController('/dev/ttyUSB0') as motor:
        motor.release_emergency_stop()
        motor.set_speed(200)        # 20% forward
        time.sleep(3)
        motor.stop()
"""

import time
import serial
from dataclasses import dataclass
from typing import Optional, Union


# ---------------------------------------------------------------------------
# Data classes returned by sensor queries
# ---------------------------------------------------------------------------

@dataclass
class Voltages:
    """Internal voltages reported by the controller."""
    internal_v: float    # Driver stage voltage (Volts)
    battery_v: float     # Main battery voltage (Volts)
    output_5v: float     # 5 V output on DSub connector (Volts)


@dataclass
class Temperatures:
    """Heatsink and IC temperatures in degrees Celsius."""
    internal_ic: int        # Internal silicon die
    channel1_heatsink: int  # Channel 1 heatsink side
    channel2_heatsink: int  # Channel 2 heatsink side (0 if single-channel)


@dataclass
class FaultFlags:
    """Latched fault conditions — any True means a fault is active."""
    overheat: bool
    overvoltage: bool
    undervoltage: bool
    short_circuit: bool
    emergency_stop: bool
    sepex_excitation_fault: bool
    mosfet_failure: bool
    startup_config_fault: bool

    def any_fault(self) -> bool:
        """Return True if any fault bit is set."""
        return any([
            self.overheat, self.overvoltage, self.undervoltage,
            self.short_circuit, self.emergency_stop,
            self.sepex_excitation_fault, self.mosfet_failure,
            self.startup_config_fault,
        ])


@dataclass
class StatusFlags:
    """Real-time controller status bits."""
    serial_mode: bool
    pulse_mode: bool
    analog_mode: bool
    power_stage_off: bool
    stall_detected: bool
    at_limit: bool
    micro_basic_running: bool


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------

class KeyaError(Exception):
    """Raised when the controller rejects a command or the connection fails."""


# ---------------------------------------------------------------------------
# Main controller class
# ---------------------------------------------------------------------------

class KeyaMotorController:
    """
    Interface to a Keya brushed/brushless motor controller over RS232.

    Supports single- and dual-channel controllers.  All speed/power values
    use the controller's native scale: -1000 (full reverse) to +1000 (full
    forward), where ±1000 equals 100 % duty cycle.

    Args:
        port:    Serial device path, e.g. '/dev/ttyUSB0' or 'COM3'.
        timeout: Per-read timeout in seconds (default 1.0).
        debug:   Print every TX/RX line to stdout when True (default False).

    Examples:
        # Context-manager (recommended) — auto connect/disconnect:
        with KeyaMotorController('/dev/ttyUSB0') as m:
            m.set_speed(300)
            time.sleep(2)
            m.stop()

        # Manual connect/disconnect:
        m = KeyaMotorController('/dev/ttyUSB0')
        m.connect()
        m.set_speed(300)
        m.disconnect()
    """

    BAUD_RATE = 115200

    def __init__(self, port: str = '/dev/ttyUSB0', timeout: float = 1.0, debug: bool = False):
        self.port = port
        self.timeout = timeout
        self.debug = debug          # print TX/RX lines when True
        self._serial: Optional[serial.Serial] = None

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open the serial port and prepare the connection."""
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            stopbits=serial.STOPBITS_ONE,
            parity=serial.PARITY_NONE,
            timeout=self.timeout,
        )
        time.sleep(0.05)  # brief settle after open

    def disconnect(self) -> None:
        """Close the serial port."""
        if self._serial and self._serial.is_open:
            self._serial.close()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()

    @property
    def is_connected(self) -> bool:
        """True if the serial port is open."""
        return bool(self._serial and self._serial.is_open)

    # ------------------------------------------------------------------
    # Low-level send / receive
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Debug helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _describe_tx(cmd: str) -> str:
        """Return a short human-readable description of an outgoing command."""
        p = cmd.split()
        u = cmd.upper().strip()

        # --- motor control ---
        if u == '!MG':
            return 'Release emergency stop'
        if u == '!EX':
            return 'Emergency stop'
        if u.startswith('!MS'):
            ch = p[1] if len(p) > 1 else 'all'
            return f'Stop channel {ch}'
        if u.startswith('!M '):
            desc = f'Set motor speed: ch1={int(p[1]):+d} ({abs(int(p[1]))/10:.0f}%)'
            if len(p) > 2:
                desc += f'  ch2={int(p[2]):+d} ({abs(int(p[2]))/10:.0f}%)'
            return desc
        if u.startswith('!P '):
            return f'Go to absolute position: ch={p[1]}  target={p[2]} counts'
        # --- config ---
        if u.startswith('^MAC'):
            return f'Set acceleration: ch={p[1]}  {p[2]} × 0.1 RPM/s'
        if u.startswith('^MDEC'):
            return f'Set deceleration: ch={p[1]}  {p[2]} × 0.1 RPM/s'
        if u.startswith('^MVEL'):
            return f'Set position velocity: ch={p[1]}  {p[2]} RPM'
        if u == '%EESAV':
            return 'Save configuration to EEPROM'
        # --- digital I/O ---
        if u.startswith('!D0'):
            return f'Turn OFF digital output {p[1]}'
        if u.startswith('!D1'):
            return f'Turn ON  digital output {p[1]}'
        if u.startswith('!DS'):
            return f'Set all digital outputs: bitmask={p[1]} (binary {int(p[1]):08b})'
        # --- queries ---
        _queries = {
            '?V':   'Read voltages (internal / battery / 5 V)',
            '?FF':  'Read fault flags',
            '?FS':  'Read status flags',
            '?LK':  'Read lock status',
            '?E':   'Read closed-loop error',
            '?DO':  'Read digital output states',
            '?BS':  'Read brushless motor speed (Hall sensors)',
            '?CIA': 'Read internal analog command value',
            '?CIP': 'Read internal pulse command value',
            '?CIS': 'Read internal serial command value',
        }
        base = u.split()[0]
        if base in _queries:
            return _queries[base]
        if base == '?A':
            ch = f' ch{p[1]}' if len(p) > 1 else ''
            return f'Read motor amps{ch}'
        if base == '?AIC':
            return 'Read analog input after conversion (±1000 scale)'
        if base == '?AI':
            ch = f' ch{p[1]}' if len(p) > 1 else ''
            return f'Read analog input{ch} (raw mV)'
        if base == '?BA':
            ch = f' ch{p[1]}' if len(p) > 1 else ''
            return f'Read battery amps{ch}'
        if base == '?C':
            ch = f' ch{p[1]}' if len(p) > 1 else ''
            return f'Read encoder position{ch} (counts)'
        if base == '?DI':
            ch = f' {p[1]}' if len(p) > 1 else ''
            return f'Read digital input{ch}'
        if base == '?D':
            return 'Read all digital inputs (bitmask)'
        if base == '?F':
            ch = f' ch{p[1]}' if len(p) > 1 else ''
            return f'Read feedback{ch}'
        if base == '?M':
            ch = f' ch{p[1]}' if len(p) > 1 else ''
            return f'Read applied motor command{ch} (±1000)'
        if base == '?P':
            ch = f' ch{p[1]}' if len(p) > 1 else ''
            return f'Read applied power level{ch} (±1000)'
        if base == '?PIC':
            return 'Read pulse input after conversion (±1000 scale)'
        if base == '?PI':
            ch = f' ch{p[1]}' if len(p) > 1 else ''
            return f'Read pulse input{ch} (µs)'
        if base == '?S':
            ch = f' ch{p[1]}' if len(p) > 1 else ''
            return f'Read encoder speed{ch} (RPM)'
        if base == '?T':
            ch = f' ch{p[1]}' if len(p) > 1 else ''
            return f'Read temperatures{ch} (°C)'
        return ''

    @staticmethod
    def _describe_rx(resp: str, sent_cmd: str) -> str:
        """Return a short human-readable description of an incoming response."""
        if resp == '+':
            return 'OK — acknowledged'
        if resp == '-':
            return 'ERROR — command not recognised'

        key = resp.split('=')[0].upper() if '=' in resp else ''
        val = resp.split('=', 1)[1] if '=' in resp else ''

        if key == 'V':
            parts = val.split(':')
            if len(parts) == 3:
                return (f'internal={int(parts[0])/10:.1f} V  '
                        f'battery={int(parts[1])/10:.1f} V  '
                        f'5Vout={int(parts[2])/1000:.3f} V')
        if key == 'FF':
            bits = int(val)
            if bits == 0:
                return 'no faults active'
            names = ['overheat', 'overvoltage', 'undervoltage', 'short-circuit',
                     'e-stop', 'sepex-fault', 'MOSFET-fail', 'startup-fault']
            active = [names[i] for i in range(8) if bits & (1 << i)]
            return 'FAULT: ' + ', '.join(active)
        if key == 'FS':
            bits = int(val)
            names = ['serial', 'pulse', 'analog', 'pwr-stage-off',
                     'stall', 'at-limit', '(unused)', 'uBasic-running']
            active = [names[i] for i in range(8) if bits & (1 << i)]
            return 'active: ' + (', '.join(active) if active else 'none')
        if key == 'A':
            parts = val.split(':')
            return '  '.join(f'ch{i+1}={int(v)/10:.1f} A' for i, v in enumerate(parts))
        if key == 'BA':
            parts = val.split(':')
            return '  '.join(f'ch{i+1}={int(v)/10:.1f} A (battery)' for i, v in enumerate(parts))
        if key == 'M':
            parts = val.split(':')
            return '  '.join(f'ch{i+1}={int(v):+d}/1000 ({abs(int(v))/10:.0f}%)' for i, v in enumerate(parts))
        if key == 'P':
            parts = val.split(':')
            return '  '.join(f'ch{i+1}={int(v):+d}/1000 power' for i, v in enumerate(parts))
        if key == 'S':
            parts = val.split(':')
            return '  '.join(f'ch{i+1}={v} RPM' for i, v in enumerate(parts))
        if key == 'BS':
            return f'{val} RPM (Hall sensor)'
        if key == 'T':
            parts = val.split(':')
            labels = ['IC', 'ch1-heatsink', 'ch2-heatsink']
            return '  '.join(f'{labels[i]}={v} °C' for i, v in enumerate(parts) if i < len(labels))
        if key == 'C':
            return f'encoder count = {val}'
        if key == 'E':
            return f'closed-loop error = {val}'
        if key == 'F':
            return f'feedback = {val}'
        if key == 'LK':
            return 'locked' if val == '1' else 'unlocked'
        if key == 'AI':
            return f'{val} mV (raw)'
        if key == 'AIC':
            return f'converted = {val} (±1000 scale)'
        if key in ('CIA', 'CIP', 'CIS'):
            src = {'CIA': 'analog', 'CIP': 'pulse', 'CIS': 'serial'}[key]
            return f'{src} command = {val} (±1000)'
        if key == 'DI':
            return f'state = {val}  (1=active 0=inactive)'
        if key == 'DO':
            return f'outputs bitmask = {val}  (binary {int(val):08b})'
        if key == 'D':
            return f'all inputs bitmask = {val}'
        if key == 'PI':
            return f'{val} µs (raw pulse width)'
        if key == 'PIC':
            return f'converted = {val} (±1000 scale)'
        return ''

    # ------------------------------------------------------------------
    # Low-level send / receive
    # ------------------------------------------------------------------

    def _send(self, command: str) -> str:
        """
        Send *command* (no trailing \\r needed) and return the stripped response.

        Raises:
            KeyaError: If not connected or the controller replies with '-'.
        """
        if not self.is_connected:
            raise KeyaError("Not connected — call connect() first.")

        self._serial.reset_input_buffer()
        self._serial.write((command + '\r').encode('ascii'))

        if self.debug:
            desc = self._describe_tx(command)
            print(f"  TX  {command:<22}  {desc}")

        # The controller echoes the command before sending the actual response.
        # Read lines until we get something that isn't our own echo.
        for _ in range(4):
            raw = self._serial.read_until(b'\r')
            response = raw.decode('ascii', errors='replace').strip()

            if self.debug:
                if response.upper() == command.upper():
                    print(f"  RX  {response:<22}  [echo]")
                else:
                    desc = self._describe_rx(response, command)
                    print(f"  RX  {response:<22}  {desc}")

            if response.upper() == command.upper():
                continue  # echo — read the real response

            if response == '-':
                raise KeyaError(f"Controller rejected command: {command!r}")

            return response

        raise KeyaError(f"No response after echo for command: {command!r}")

    def _write(self, command: str) -> None:
        """Send a write/action command and verify the '+' acknowledgment."""
        resp = self._send(command)
        if resp and resp != '+':
            raise KeyaError(
                f"Unexpected ack for command {command!r}: {resp!r}"
            )

    def _query(self, command: str) -> str:
        """Send a query command and return the value after '='."""
        resp = self._send(command)
        if '=' not in resp:
            raise KeyaError(
                f"Unexpected response for query {command!r}: {resp!r}"
            )
        return resp.split('=', 1)[1]

    # ------------------------------------------------------------------
    # Motor control
    # ------------------------------------------------------------------

    def set_speed(
        self,
        channel1: int,
        channel2: Optional[int] = None,
    ) -> None:
        """
        Set motor speed.

        Args:
            channel1: Speed for channel 1, range -1000 to +1000.
                      Positive = forward, negative = reverse.
            channel2: Speed for channel 2 (dual-channel controllers only).
                      Omit for single-channel controllers.

        Raises:
            ValueError: Speed out of -1000..+1000 range.

        Examples:
            motor.set_speed(500)          # 50 % forward, channel 1
            motor.set_speed(500, -300)    # Ch1 fwd 50 %, Ch2 rev 30 %
        """
        if not -1000 <= channel1 <= 1000:
            raise ValueError(f"channel1 speed {channel1} out of range -1000..1000")
        if channel2 is not None:
            if not -1000 <= channel2 <= 1000:
                raise ValueError(f"channel2 speed {channel2} out of range -1000..1000")
            self._write(f'!M {channel1} {channel2}')
        else:
            self._write(f'!M {channel1}')

    def stop(self, channel: Optional[int] = None) -> None:
        """
        Stop motor(s) by setting speed to zero.

        Args:
            channel: Channel to stop (1 or 2).  If None, stops all channels.
        """
        if channel is not None:
            self._write(f'!MS {channel}')
        else:
            self.set_speed(0)

    def emergency_stop(self) -> None:
        """
        Trigger a software emergency stop.

        The controller will not accept motion commands until
        :meth:`release_emergency_stop` is called or the unit is power-cycled.
        """
        self._write('!EX')

    def release_emergency_stop(self) -> None:
        """Release a software emergency stop and resume normal operation."""
        self._write('!MG')

    def go_to_position(self, position: int, channel: int = 1) -> None:
        """
        Move to an absolute encoder position (position-count mode only).

        Args:
            position: Target encoder count.  One revolution = encoder_ppr × 4.
                      Range: ±2 147 483 647.
            channel:  Motor channel (default 1).
        """
        if not -2_147_483_647 <= position <= 2_147_483_647:
            raise ValueError("Position out of ±2147483647 range")
        self._write(f'!P {channel} {position}')

    # ------------------------------------------------------------------
    # Configuration (runtime-settable, saved with save_config())
    # ------------------------------------------------------------------

    def set_acceleration(self, rate: int, channel: int = 1) -> None:
        """
        Set motor acceleration ramp rate.

        Args:
            rate:    Acceleration in 0.1 RPM/s steps.  Range: 100–32 000.
            channel: Motor channel (default 1).
        """
        if not 100 <= rate <= 32_000:
            raise ValueError(f"Acceleration {rate} out of range 100..32000")
        self._write(f'^MAC {channel} {rate}')

    def set_deceleration(self, rate: int, channel: int = 1) -> None:
        """
        Set motor deceleration ramp rate.

        Args:
            rate:    Deceleration in 0.1 RPM/s steps.  Range: 100–32 000.
            channel: Motor channel (default 1).
        """
        if not 100 <= rate <= 32_000:
            raise ValueError(f"Deceleration {rate} out of range 100..32000")
        self._write(f'^MDEC {channel} {rate}')

    def set_position_velocity(self, rpm: int, channel: int = 1) -> None:
        """
        Set default velocity used when in position mode.

        Args:
            rpm:     Target speed in RPM.
            channel: Motor channel (default 1).
        """
        self._write(f'^MVEL {channel} {rpm}')

    def save_config(self) -> None:
        """
        Save current configuration to EEPROM (survives power-off).

        Warning: Do NOT call while motors are running — the control loop
        pauses for several milliseconds during the write.
        """
        self._write('%EESAV')

    # ------------------------------------------------------------------
    # Digital I/O
    # ------------------------------------------------------------------

    def set_digital_output(self, pin: int, state: bool) -> None:
        """
        Set a single digital output pin ON or OFF.

        Args:
            pin:   Output pin number (controller-specific, typically 1-based).
            state: True = ON, False = OFF.
        """
        cmd = f'!D1 {pin}' if state else f'!D0 {pin}'
        self._write(cmd)

    def set_all_digital_outputs(self, pattern: int) -> None:
        """
        Set all digital outputs simultaneously via a bitmask.

        Args:
            pattern: Integer 0–255.  Bit 0 controls output 1, bit 1 controls
                     output 2, etc.  Example: 3 (0b00000011) → outputs 1 & 2 ON.
        """
        if not 0 <= pattern <= 255:
            raise ValueError("Bit pattern must be 0–255")
        self._write(f'!DS {pattern}')

    def read_digital_outputs(self) -> int:
        """
        Read current state of all digital outputs as a bitmask.

        Returns:
            Integer; convert to binary to inspect individual output bits.
        """
        return int(self._query('?DO'))

    def read_digital_input(self, pin: int) -> bool:
        """Read a single digital input pin. Returns True if active."""
        return bool(int(self._query(f'?DI {pin}')))

    def read_all_digital_inputs(self) -> list[int]:
        """
        Read all digital inputs.

        Returns:
            List of 0/1 integers, one per available input channel.
        """
        val = self._query('?DI')
        return [int(x) for x in val.split(':')]

    # ------------------------------------------------------------------
    # Sensor / telemetry queries
    # ------------------------------------------------------------------

    def read_voltages(self) -> Voltages:
        """
        Read internal controller voltages.

        Returns:
            :class:`Voltages` with driver, battery, and 5 V output readings.

        Example:
            v = motor.read_voltages()
            print(f"Battery: {v.battery_v:.1f} V")
        """
        parts = self._query('?V').split(':')
        return Voltages(
            internal_v=int(parts[0]) / 10.0,
            battery_v=int(parts[1]) / 10.0,
            output_5v=int(parts[2]) / 1000.0,
        )

    def read_temperatures(self) -> Temperatures:
        """
        Read heatsink and IC temperatures in degrees Celsius.

        Returns:
            :class:`Temperatures` dataclass.
        """
        parts = self._query('?T').split(':')
        return Temperatures(
            internal_ic=int(parts[0]),
            channel1_heatsink=int(parts[1]) if len(parts) > 1 else 0,
            channel2_heatsink=int(parts[2]) if len(parts) > 2 else 0,
        )

    def read_motor_amps(self, channel: Optional[int] = None) -> Union[float, list[float]]:
        """
        Read motor current in Amperes.

        Args:
            channel: Motor channel.  If None, reads all channels.

        Returns:
            Single float, or list of floats for multi-channel controllers.
        """
        cmd = f'?A {channel}' if channel else '?A'
        parts = [int(p) / 10.0 for p in self._query(cmd).split(':')]
        return parts[0] if len(parts) == 1 else parts

    def read_battery_amps(self, channel: Optional[int] = None) -> Union[float, list[float]]:
        """
        Read battery current in Amperes.

        Args:
            channel: Motor channel.  If None, reads all channels.

        Returns:
            Single float, or list of floats for multi-channel controllers.
        """
        cmd = f'?BA {channel}' if channel else '?BA'
        parts = [int(p) / 10.0 for p in self._query(cmd).split(':')]
        return parts[0] if len(parts) == 1 else parts

    def read_encoder_speed(self, channel: Optional[int] = None) -> Union[int, list[int]]:
        """
        Read encoder-measured motor speed in RPM.

        Args:
            channel: Motor channel.  If None, reads all channels.

        Returns:
            Speed in RPM, or list for multi-channel controllers.
        """
        cmd = f'?S {channel}' if channel else '?S'
        parts = [int(p) for p in self._query(cmd).split(':')]
        return parts[0] if len(parts) == 1 else parts

    def read_brushless_speed(self) -> int:
        """Read brushless motor speed in RPM (Hall-sensor based)."""
        return int(self._query('?BS'))

    def read_encoder_position(self, channel: Optional[int] = None) -> int:
        """
        Read the absolute encoder counter value.

        Returns:
            32-bit signed count.  One full revolution = encoder_ppr × 4.
        """
        cmd = f'?C {channel}' if channel else '?C'
        return int(self._query(cmd))

    def read_motor_command(self, channel: Optional[int] = None) -> Union[int, list[int]]:
        """
        Read the command value currently applied by the controller (-1000..+1000).

        Returns:
            Integer command, or list for multi-channel controllers.
        """
        cmd = f'?M {channel}' if channel else '?M'
        parts = [int(p) for p in self._query(cmd).split(':')]
        return parts[0] if len(parts) == 1 else parts

    def read_motor_power(self, channel: Optional[int] = None) -> Union[int, list[int]]:
        """
        Read actual power output applied to the motor (-1000..+1000).

        This reflects internal corrections and current/temperature limiting.

        Returns:
            Power level, or list for multi-channel controllers.
        """
        cmd = f'?P {channel}' if channel else '?P'
        parts = [int(p) for p in self._query(cmd).split(':')]
        return parts[0] if len(parts) == 1 else parts

    def read_analog_input(self, channel: Optional[int] = None) -> Union[int, list[int]]:
        """
        Read raw analog input(s) in millivolts (0–5000 mV).

        Args:
            channel: Input channel number.  If None, reads all.

        Returns:
            Integer millivolt reading, or list for multiple channels.
        """
        cmd = f'?AI {channel}' if channel else '?AI'
        parts = [int(p) for p in self._query(cmd).split(':')]
        return parts[0] if len(parts) == 1 else parts

    def read_closed_loop_error(self) -> int:
        """
        Read closed-loop tracking error (desired minus measured).

        Returns 0 in open-loop mode.
        """
        return int(self._query('?E'))

    def read_feedback(self, channel: Optional[int] = None) -> Union[int, list[int]]:
        """Read the feedback sensor value(s) used in closed-loop mode."""
        cmd = f'?F {channel}' if channel else '?F'
        parts = [int(p) for p in self._query(cmd).split(':')]
        return parts[0] if len(parts) == 1 else parts

    def read_fault_flags(self) -> FaultFlags:
        """
        Read all latched fault conditions.

        Returns:
            :class:`FaultFlags` — call ``.any_fault()`` for a quick check.

        Example:
            faults = motor.read_fault_flags()
            if faults.any_fault():
                print("Fault detected:", faults)
        """
        bits = int(self._query('?FF'))
        return FaultFlags(
            overheat               = bool(bits & (1 << 0)),
            overvoltage            = bool(bits & (1 << 1)),
            undervoltage           = bool(bits & (1 << 2)),
            short_circuit          = bool(bits & (1 << 3)),
            emergency_stop         = bool(bits & (1 << 4)),
            sepex_excitation_fault = bool(bits & (1 << 5)),
            mosfet_failure         = bool(bits & (1 << 6)),
            startup_config_fault   = bool(bits & (1 << 7)),
        )

    def read_status_flags(self) -> StatusFlags:
        """
        Read real-time controller status flags.

        Returns:
            :class:`StatusFlags` dataclass.
        """
        bits = int(self._query('?FS'))
        return StatusFlags(
            serial_mode      = bool(bits & (1 << 0)),
            pulse_mode       = bool(bits & (1 << 1)),
            analog_mode      = bool(bits & (1 << 2)),
            power_stage_off  = bool(bits & (1 << 3)),
            stall_detected   = bool(bits & (1 << 4)),
            at_limit         = bool(bits & (1 << 5)),
            micro_basic_running = bool(bits & (1 << 7)),
        )

    def is_locked(self) -> bool:
        """Return True if the controller configuration is locked."""
        return self._query('?LK').strip() == '1'

    def read_internal_serial_command(self) -> int:
        """Read the command value currently issued from the serial interface (-1000..+1000)."""
        return int(self._query('?CIS'))

    def read_pulse_input(self, channel: Optional[int] = None) -> Union[int, list[int]]:
        """
        Read raw pulse input value in microseconds (Pulse Width mode).

        Returns:
            Microseconds (0–65 000), or list for multiple channels.
        """
        cmd = f'?PI {channel}' if channel else '?PI'
        parts = [int(p) for p in self._query(cmd).split(':')]
        return parts[0] if len(parts) == 1 else parts
