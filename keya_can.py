"""
keya_can.py — CAN bus (CANopen SDO) transport for Keya controllers.

Reference: datasheets/kydbl4850-1e/CAN protocol.pdf

Only expedited SDO is used (no PDO/NMT; the controller autostarts by default):
    request   ID 0x600 + node_id, 8 data bytes
    response  ID 0x580 + node_id, 8 data bytes
    read   : 40 idx_lo idx_hi sub 00 00 00 00   -> 4F/4B/47/43 idx sub data...
    write  : 23 idx_lo idx_hi sub d0 d1 d2 d3   -> 60 idx sub 00 00 00 00
    abort  : 80 ...                              (bytes 4..7 = abort code)
Multi-byte values are little-endian.

The SocketCAN interface must already be up, e.g. on the Jetson:
    sudo ip link set can0 up type can bitrate 250000
(the controller's default is 250 kbit/s, node ID 1).
"""

import threading
import time
from typing import List, Optional

from keya_transport import KeyaError, KeyaUnsupportedError, Transport


class SdoAbort(KeyaError):
    """The node answered with an SDO abort (object missing, access refused...)."""


# Object dictionary -- writes: cmd -> (index, per_channel)
# (sub-index is the channel for per-channel objects, 0 otherwise)
_WRITE = {
    'MS':    (0x200E, True),    # stop channel
    'P':     (0x2001, True),    # go to absolute position
    'MVEL':  (0x2002, True),    # position-mode speed
    'MAC':   (0x2006, True),    # acceleration
    'MDEC':  (0x2007, True),    # deceleration
    'EX':    (0x200C, False),   # emergency stop
    'MG':    (0x200D, False),   # release emergency stop
    'EESAV': (0x2017, False),   # save configuration
}
_SPEED_INDEX = 0x2000           # 'M': sub 1 / sub 2 = channel 1 / 2

# Object dictionary -- reads: cmd -> (index, n_subs, n_required, signed)
# n_subs == 0 means a single object at sub-index 0.  Subs beyond n_required
# are optional (e.g. channel 2 on a single-channel controller).
_QUERY = {
    'A':   (0x2100, 2, 1, True),    # motor amps x10
    'M':   (0x2101, 2, 1, True),    # applied motor command
    'S':   (0x2103, 2, 1, True),    # encoder speed, RPM
    'C':   (0x2104, 2, 1, True),    # encoder counter (absolute)
    'BS':  (0x210A, 2, 1, True),    # brushless speed, RPM
    'BA':  (0x210C, 2, 1, True),    # battery amps x10
    'V':   (0x210D, 3, 3, False),   # internal x10, battery x10, 5 V in mV
    'T':   (0x210F, 3, 2, True),    # MCU, channel 1, channel 2 (degrees C)
    'D':   (0x210E, 0, 0, False),   # digital inputs bitmask
    'FS':  (0x2111, 0, 0, False),   # status flags
    'FF':  (0x2112, 0, 0, False),   # fault flags
    'DO':  (0x2113, 0, 0, False),   # digital outputs bitmask
}


class CanTransport(Transport):
    """Keya CANopen (SDO) protocol over python-can."""

    name = 'can'

    def __init__(
        self,
        channel: str = 'can0',
        node_id: int = 1,
        timeout: float = 1.0,
        debug: bool = False,
        bustype: str = 'socketcan',
        bitrate: Optional[int] = None,
    ):
        if not 1 <= node_id <= 126:
            raise ValueError(f"node_id {node_id} out of range 1..126")
        self.channel = channel
        self.node_id = node_id
        self.timeout = timeout
        self.debug = debug
        self.bustype = bustype
        self.bitrate = bitrate      # only used by interfaces that set it (not socketcan)
        self._bus = None
        self._lock = threading.Lock()

    # -- connection ------------------------------------------------------

    def open(self) -> None:
        try:
            import can  # imported lazily: not needed for serial-only setups
        except ImportError:
            raise KeyaError("CAN support needs python-can:  pip install python-can")
        kwargs = {'channel': self.channel, 'bustype': self.bustype}
        if self.bitrate is not None:
            kwargs['bitrate'] = self.bitrate
        try:
            self._bus = can.interface.Bus(**kwargs)
        except Exception as exc:
            raise KeyaError(f"Cannot open CAN channel {self.channel!r}: {exc}")

    def close(self) -> None:
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    @property
    def is_open(self) -> bool:
        return self._bus is not None

    # -- SDO primitives --------------------------------------------------

    def _log(self, tag: str, can_id: int, data: bytes) -> None:
        if self.debug:
            print(f"  {tag}  0x{can_id:03X}  {' '.join(f'{b:02X}' for b in data)}")

    def _transfer(self, data: bytes, wait: bool = True) -> bytes:
        """Send one SDO request frame and return the matching response data.

        wait=False: do not read the reply (fire-and-forget); returns b''.  The
        late reply is dropped as a stale frame by the next transfer."""
        import can
        if self._bus is None:
            raise KeyaError("Not connected — call connect() first.")

        req_id = 0x600 + self.node_id
        resp_id = 0x580 + self.node_id
        with self._lock:
            while self._bus.recv(0) is not None:   # drop stale frames
                pass
            self._log('TX', req_id, data)
            self._bus.send(can.Message(arbitration_id=req_id, data=data,
                                       is_extended_id=False))
            if not wait:
                return b''
            deadline = time.monotonic() + self.timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise KeyaError(
                        f"No CAN response from node {self.node_id} "
                        f"(request {bytes(data[:4]).hex()})"
                    )
                msg = self._bus.recv(remaining)
                if msg is None or msg.is_error_frame or msg.arbitration_id != resp_id:
                    continue    # heartbeat (0x700+id), other nodes, errors...
                self._log('RX', msg.arbitration_id, bytes(msg.data))
                resp = bytes(msg.data).ljust(8, b'\x00')
                break

        if resp[0] == 0x80:
            code = int.from_bytes(resp[4:8], 'little')
            raise SdoAbort(
                f"SDO abort 0x{code:08X} for object "
                f"0x{data[2] << 8 | data[1]:04X}/{data[3]}"
            )
        return resp

    def _sdo_write(self, index: int, sub: int, value: int, wait: bool = True) -> None:
        data = bytes([0x23, index & 0xFF, index >> 8, sub]) + \
            value.to_bytes(4, 'little', signed=value < 0)
        resp = self._transfer(data, wait)
        if not wait:
            return
        if resp[0] != 0x60:
            raise KeyaError(f"Unexpected SDO write reply: {resp.hex()}")

    def _sdo_read(self, index: int, sub: int, signed: bool) -> int:
        data = bytes([0x40, index & 0xFF, index >> 8, sub, 0, 0, 0, 0])
        resp = self._transfer(data)
        if resp[0] & 0xE3 != 0x43 or resp[1:4] != data[1:4]:
            raise KeyaError(f"Unexpected SDO read reply: {resp.hex()}")
        size = 4 - ((resp[0] >> 2) & 3)
        return int.from_bytes(resp[4:4 + size], 'little', signed=signed)

    # -- Transport interface ---------------------------------------------

    def write(self, cmd: str, *args: int, verify: bool = True) -> None:
        if cmd == 'M':                       # speed: one SDO per channel
            for channel, speed in enumerate(args, start=1):
                self._sdo_write(_SPEED_INDEX, channel, speed, verify)
            return
        if cmd not in _WRITE:
            raise KeyaUnsupportedError(f"Command {cmd!r} is not available over CAN")
        index, per_channel = _WRITE[cmd]
        if not per_channel:
            self._sdo_write(index, 0, 1, verify)
        elif cmd == 'MS':
            self._sdo_write(index, args[0], 1, verify)
        else:
            self._sdo_write(index, args[0], args[1], verify)

    def query(self, cmd: str, *args: int) -> List[int]:
        if cmd == 'DI':                      # derived from the inputs bitmask
            if not args:
                raise KeyaUnsupportedError("Reading all digital inputs is not available over CAN")
            return [(self.query('D')[0] >> (args[0] - 1)) & 1]
        if cmd not in _QUERY:
            raise KeyaUnsupportedError(f"Query {cmd!r} is not available over CAN")

        index, n_subs, n_required, signed = _QUERY[cmd]
        if n_subs == 0:
            return [self._sdo_read(index, 0, signed)]
        if args:                             # single channel / element
            return [self._sdo_read(index, args[0], signed)]

        values = []
        for sub in range(1, n_subs + 1):
            try:
                values.append(self._sdo_read(index, sub, signed))
            except SdoAbort:
                if sub <= n_required:
                    raise
                break                        # fewer channels than the maximum
        return values
