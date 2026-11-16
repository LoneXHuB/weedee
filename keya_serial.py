"""
keya_serial.py — RS232 (ASCII) transport for Keya controllers.

Serial settings: 115200 baud, 8-N-1.
Commands are ASCII, terminated with carriage return (\\r).
Acknowledgment: '+' for write commands, response string for queries, '-' on error.
"""

import time
from typing import List, Optional

from keya_transport import KeyaError, Transport


class _Rejected(KeyaError):
    """Controller answered '-' (command not recognised or lost)."""


class SerialTransport(Transport):
    """Keya RS232 text protocol."""

    name = 'serial'
    BAUD_RATE = 115200

    # Commands that do not use the default '!' (real-time) prefix.
    _PREFIX = {'MAC': '^', 'MDEC': '^', 'MVEL': '^', 'EESAV': '%'}

    def __init__(self, port: str, timeout: float = 1.0, debug: bool = False,
                 retries: int = 1):
        self.port = port
        self.retries = retries      # resends after a '-' reply
        self.timeout = timeout
        self.debug = debug
        self._serial = None  # type: Optional[object]

    # -- connection ------------------------------------------------------

    def open(self) -> None:
        import serial  # imported lazily: not needed for CAN-only setups
        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            stopbits=serial.STOPBITS_ONE,
            parity=serial.PARITY_NONE,
            timeout=self.timeout,
        )
        time.sleep(0.05)  # brief settle after open

    def close(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()

    @property
    def is_open(self) -> bool:
        return bool(self._serial and self._serial.is_open)

    # -- debug helpers ---------------------------------------------------

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

        key = resp.split('=')[0].strip().upper() if '=' in resp else ''
        val = resp.split('=', 1)[1].strip() if '=' in resp else ''

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

    # -- low-level send / receive -----------------------------------------

    def _send(self, command: str) -> str:
        """
        Send *command* and return the response.  Per the protocol manual, a '-'
        reply means the command was not recognised *or lost*, so it is resent
        up to ``retries`` times before giving up.

        Raises:
            KeyaError: If not connected or the controller keeps replying '-'.
        """
        for attempt in range(self.retries + 1):
            try:
                return self._send_once(command)
            except _Rejected:
                if attempt == self.retries:
                    raise

    def _send_once(self, command: str) -> str:
        """Send *command* (no trailing \\r needed) once; return the stripped response."""
        if not self.is_open:
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
                raise _Rejected(f"Controller rejected command: {command!r}")

            return response

        raise KeyaError(f"No response after echo for command: {command!r}")

    # -- Transport interface ---------------------------------------------

    @classmethod
    def _format(cls, prefix: str, cmd: str, args) -> str:
        return ' '.join([prefix + cmd] + [str(a) for a in args])

    def write(self, cmd: str, *args: int) -> None:
        command = self._format(self._PREFIX.get(cmd, '!'), cmd, args)
        resp = self._send(command)
        if resp and resp != '+':
            raise KeyaError(
                f"Unexpected ack for command {command!r}: {resp!r}"
            )

    def query(self, cmd: str, *args: int) -> List[int]:
        command = self._format('?', cmd, args)
        resp = self._send(command)
        if '=' not in resp:
            raise KeyaError(
                f"Unexpected response for query {command!r}: {resp!r}"
            )
        try:
            return [int(x) for x in resp.split('=', 1)[1].split(':')]
        except ValueError:
            raise KeyaError(f"Malformed response for {command!r}: {resp!r}")
