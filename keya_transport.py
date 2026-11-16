"""
keya_transport.py — Transport abstraction shared by the Keya backends.

A transport hides the wire protocol (RS232 text or CAN/CANopen) behind two
operations expressed with the controller's own command mnemonics:

    write(cmd, *args)   -> None         e.g. write('M', 500), write('MS', 1)
    query(cmd, *args)   -> List[int]    e.g. query('V') -> [135, 246, 4730]

Values are raw controller integers; scaling and dataclasses are handled by
:class:`keya_motor.KeyaMotorController`.
"""

from typing import List


class KeyaError(Exception):
    """Raised when the controller rejects a command or the connection fails."""


class KeyaUnsupportedError(KeyaError):
    """Raised when the selected protocol does not offer the requested feature."""


class Transport:
    """Interface implemented by SerialTransport and CanTransport."""

    #: Short protocol name, e.g. 'serial' or 'can'.
    name = ''

    def open(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    @property
    def is_open(self) -> bool:
        raise NotImplementedError

    def write(self, cmd: str, *args: int, verify: bool = True) -> None:
        """Send a command.  verify=False: fire-and-forget, do not read the reply
        (for high-rate live control; link noise then cannot raise)."""
        raise NotImplementedError

    def query(self, cmd: str, *args: int) -> List[int]:
        raise NotImplementedError
