"""
keya_motor.py — Python library for Keya motor controllers (RS232 or CAN bus).

The wire protocol is hidden: the same API works over RS232 (ASCII) and over
CAN (CANopen SDO).  The protocol is picked from the port name, or explicitly.

All speed/power values use the controller's native scale: -1000 (full
reverse) to +1000 (full forward).

Typical usage:
    from keya_motor import KeyaMotorController

    with KeyaMotorController('/dev/ttyUSB0') as motor:     # RS232
    # with KeyaMotorController('can0', node_id=1) as motor:  # CAN
        motor.release_emergency_stop()
        motor.set_speed(200)        # 20% forward
        time.sleep(3)
        motor.stop()

Features missing on a given protocol (e.g. digital outputs over CAN) raise
:class:`KeyaUnsupportedError`, a subclass of :class:`KeyaError`.
"""

from dataclasses import dataclass
from typing import List, Optional, Union

from keya_transport import KeyaError, KeyaUnsupportedError, Transport

__all__ = ['KeyaMotorController', 'KeyaError', 'KeyaUnsupportedError',
           'Voltages', 'Temperatures', 'FaultFlags', 'StatusFlags']


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
# Main controller class
# ---------------------------------------------------------------------------

class KeyaMotorController:
    """
    Interface to a Keya brushed/brushless motor controller.

    Supports single- and dual-channel controllers over RS232 or CAN.

    Args:
        port:     Serial device path ('/dev/ttyUSB0', 'COM3') or CAN channel
                  ('can0').
        timeout:  Per-read timeout in seconds (default 1.0).
        debug:    Print every TX/RX exchange to stdout when True.
        protocol: 'auto' (default: 'can*'/'vcan*'/'slcan*' ports use CAN,
                  everything else RS232), 'serial' or 'can'.
        node_id:  CAN node ID of the controller, 1..126 (CAN only, default 1).
        **can_options: extra CAN settings (CAN only): ``bustype``
                  (default 'socketcan'), ``bitrate``.

    Examples:
        # Context-manager (recommended) — auto connect/disconnect:
        with KeyaMotorController('/dev/ttyUSB0') as m:
            m.set_speed(300)
            time.sleep(2)
            m.stop()

        # Same API over CAN:
        with KeyaMotorController('can0', node_id=1) as m:
            m.set_speed(300)
            print(m.protocol)   # 'can'
    """

    _CAN_PREFIXES = ('can', 'vcan', 'slcan')

    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        timeout: float = 1.0,
        debug: bool = False,
        protocol: str = 'auto',
        node_id: int = 1,
        **can_options,
    ):
        if protocol == 'auto':
            protocol = 'can' if port.lower().startswith(self._CAN_PREFIXES) else 'serial'
        self.port = port
        self.timeout = timeout
        self.debug = debug
        self._transport = self._make_transport(protocol, node_id, can_options)  # type: Transport

    def _make_transport(self, protocol: str, node_id: int, can_options: dict) -> Transport:
        if protocol == 'serial':
            from keya_serial import SerialTransport
            return SerialTransport(self.port, self.timeout, self.debug)
        if protocol == 'can':
            from keya_can import CanTransport
            return CanTransport(self.port, node_id, self.timeout, self.debug, **can_options)
        raise ValueError(f"Unknown protocol {protocol!r} (use 'auto', 'serial' or 'can')")

    @property
    def protocol(self) -> str:
        """Active wire protocol: 'serial' or 'can'."""
        return self._transport.name

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open the serial port / CAN channel."""
        self._transport.open()

    def disconnect(self) -> None:
        """Close the serial port / CAN channel."""
        self._transport.close()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()

    @property
    def is_connected(self) -> bool:
        """True if the link is open."""
        return self._transport.is_open

    # ------------------------------------------------------------------
    # Protocol-neutral helpers
    # ------------------------------------------------------------------

    def _write(self, cmd: str, *args: int) -> None:
        self._transport.write(cmd, *args)

    def _query(self, cmd: str, *args: int) -> List[int]:
        return self._transport.query(cmd, *args)

    def _scalar(self, cmd: str, *args: int) -> int:
        return self._query(cmd, *args)[0]

    def _per_channel(self, cmd: str, channel: Optional[int], scale: float = 1):
        """Query one value (channel given) or all channels; unwrap singletons."""
        parts = self._query(cmd, *([channel] if channel else []))
        if scale != 1:
            parts = [p / scale for p in parts]
        return parts[0] if len(parts) == 1 else parts

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
            self._write('M', channel1, channel2)
        else:
            self._write('M', channel1)

    def stop(self, channel: Optional[int] = None) -> None:
        """
        Stop motor(s) by setting speed to zero.

        Args:
            channel: Channel to stop (1 or 2).  If None, stops all channels.
        """
        if channel is not None:
            self._write('MS', channel)
            return
        try:                                  # zero every channel the controller has
            n_channels = max(1, len(self._query('M')))
        except KeyaUnsupportedError:
            n_channels = 1
        self._write('M', *([0] * n_channels))

    def emergency_stop(self) -> None:
        """
        Trigger a software emergency stop.

        The controller will not accept motion commands until
        :meth:`release_emergency_stop` is called or the unit is power-cycled.
        """
        self._write('EX')

    def release_emergency_stop(self) -> None:
        """Release a software emergency stop and resume normal operation."""
        self._write('MG')

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
        self._write('P', channel, position)

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
        self._write('MAC', channel, rate)

    def set_deceleration(self, rate: int, channel: int = 1) -> None:
        """
        Set motor deceleration ramp rate.

        Args:
            rate:    Deceleration in 0.1 RPM/s steps.  Range: 100–32 000.
            channel: Motor channel (default 1).
        """
        if not 100 <= rate <= 32_000:
            raise ValueError(f"Deceleration {rate} out of range 100..32000")
        self._write('MDEC', channel, rate)

    def set_position_velocity(self, rpm: int, channel: int = 1) -> None:
        """
        Set default velocity used when in position mode.

        Args:
            rpm:     Target speed in RPM.
            channel: Motor channel (default 1).
        """
        self._write('MVEL', channel, rpm)

    def save_config(self) -> None:
        """
        Save current configuration to EEPROM (survives power-off).

        Warning: Do NOT call while motors are running — the control loop
        pauses for several milliseconds during the write.
        """
        self._write('EESAV')

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
        self._write('D1' if state else 'D0', pin)

    def set_all_digital_outputs(self, pattern: int) -> None:
        """
        Set all digital outputs simultaneously via a bitmask.

        Args:
            pattern: Integer 0–255.  Bit 0 controls output 1, bit 1 controls
                     output 2, etc.  Example: 3 (0b00000011) → outputs 1 & 2 ON.
        """
        if not 0 <= pattern <= 255:
            raise ValueError("Bit pattern must be 0–255")
        self._write('DS', pattern)

    def read_digital_outputs(self) -> int:
        """
        Read current state of all digital outputs as a bitmask.

        Returns:
            Integer; convert to binary to inspect individual output bits.
        """
        return self._scalar('DO')

    def read_digital_input(self, pin: int) -> bool:
        """Read a single digital input pin. Returns True if active."""
        return bool(self._scalar('DI', pin))

    def read_all_digital_inputs(self) -> List[int]:
        """
        Read all digital inputs.

        Returns:
            List of 0/1 integers, one per available input channel.
        """
        return self._query('DI')

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
        parts = self._query('V')
        return Voltages(
            internal_v=parts[0] / 10.0,
            battery_v=parts[1] / 10.0,
            output_5v=parts[2] / 1000.0,
        )

    def read_temperatures(self) -> Temperatures:
        """
        Read heatsink and IC temperatures in degrees Celsius.

        Returns:
            :class:`Temperatures` dataclass.
        """
        parts = self._query('T')
        return Temperatures(
            internal_ic=parts[0],
            channel1_heatsink=parts[1] if len(parts) > 1 else 0,
            channel2_heatsink=parts[2] if len(parts) > 2 else 0,
        )

    def read_motor_amps(self, channel: Optional[int] = None) -> Union[float, List[float]]:
        """
        Read motor current in Amperes.

        Args:
            channel: Motor channel.  If None, reads all channels.

        Returns:
            Single float, or list of floats for multi-channel controllers.
        """
        return self._per_channel('A', channel, 10.0)

    def read_battery_amps(self, channel: Optional[int] = None) -> Union[float, List[float]]:
        """
        Read battery current in Amperes.

        Args:
            channel: Motor channel.  If None, reads all channels.

        Returns:
            Single float, or list of floats for multi-channel controllers.
        """
        return self._per_channel('BA', channel, 10.0)

    def read_encoder_speed(self, channel: Optional[int] = None) -> Union[int, List[int]]:
        """
        Read encoder-measured motor speed in RPM.

        Args:
            channel: Motor channel.  If None, reads all channels.

        Returns:
            Speed in RPM, or list for multi-channel controllers.
        """
        return self._per_channel('S', channel)

    def read_brushless_speed(self) -> int:
        """Read brushless motor speed in RPM (Hall-sensor based)."""
        return self._scalar('BS')

    def read_encoder_position(self, channel: Optional[int] = None) -> int:
        """
        Read the absolute encoder counter value.

        Returns:
            32-bit signed count.  One full revolution = encoder_ppr × 4.
        """
        return self._per_channel('C', channel)

    def read_motor_command(self, channel: Optional[int] = None) -> Union[int, List[int]]:
        """
        Read the command value currently applied by the controller (-1000..+1000).

        Returns:
            Integer command, or list for multi-channel controllers.
        """
        return self._per_channel('M', channel)

    def read_motor_power(self, channel: Optional[int] = None) -> Union[int, List[int]]:
        """
        Read actual power output applied to the motor (-1000..+1000).

        This reflects internal corrections and current/temperature limiting.

        Returns:
            Power level, or list for multi-channel controllers.
        """
        return self._per_channel('P', channel)

    def read_analog_input(self, channel: Optional[int] = None) -> Union[int, List[int]]:
        """
        Read raw analog input(s) in millivolts (0–5000 mV).

        Args:
            channel: Input channel number.  If None, reads all.

        Returns:
            Integer millivolt reading, or list for multiple channels.
        """
        return self._per_channel('AI', channel)

    def read_closed_loop_error(self) -> int:
        """
        Read closed-loop tracking error (desired minus measured).

        Returns 0 in open-loop mode.
        """
        return self._scalar('E')

    def read_feedback(self, channel: Optional[int] = None) -> Union[int, List[int]]:
        """Read the feedback sensor value(s) used in closed-loop mode."""
        return self._per_channel('F', channel)

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
        bits = self._scalar('FF')
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
        bits = self._scalar('FS')
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
        return self._scalar('LK') == 1

    def read_internal_serial_command(self) -> int:
        """Read the command value currently issued from the serial interface (-1000..+1000)."""
        return self._scalar('CIS')

    def read_internal_analog_command(self) -> int:
        """Read the command computed from the analog inputs, applied or not (-1000..+1000)."""
        return self._scalar('CIA')

    def read_internal_pulse_command(self) -> int:
        """Read the command computed from the pulse inputs, applied or not (-1000..+1000)."""
        return self._scalar('CIP')

    def read_analog_input_converted(self) -> int:
        """Read the analog input after Min/Max/Center/Deadband/Linearity conversion (-1000..+1000)."""
        return self._scalar('AIC')

    def read_pulse_input_converted(self) -> int:
        """Read the pulse input after Min/Max/Center/Deadband/Linearity conversion (-1000..+1000)."""
        return self._scalar('PIC')

    def read_digital_inputs_mask(self) -> int:
        """Read all digital inputs as a bitmask (bit 0 = input 1)."""
        return self._scalar('D')

    def read_pulse_input(self, channel: Optional[int] = None) -> Union[int, List[int]]:
        """
        Read raw pulse input capture.

        Returns:
            Microseconds (0–65 000) in Pulse Width mode, Hertz in Frequency
            mode, 0–4095 in Duty Cycle mode; list for multiple channels.
        """
        return self._per_channel('PI', channel)
