"""pyserial stand-in for the Celigo driver's pylibftdi transport.

``Celigo`` talks to its controller through ``pylabrobot.io.ftdi.FTDI``, which needs libftdi,
libusb, and a WinUSB/libusbK driver bound with Zadig. This adapter implements the same
calls on the FTDI virtual COM port instead, so an instrument that Windows already lists
as ``COMn`` can be evaluated without changing drivers.

The FTDI latency timer cannot be set through a COM port. Set it in Device Manager
(Port Settings -> Advanced -> Latency Timer); the driver asks for 2 ms.
"""

import asyncio
import logging
from concurrent.futures import ThreadPoolExecutor
from typing import Optional

import serial

logger = logging.getLogger(__name__)

# libftdi ftdi_set_line_property enum values -> pyserial constants.
_STOP_BITS = {0: serial.STOPBITS_ONE, 1: serial.STOPBITS_ONE_POINT_FIVE, 2: serial.STOPBITS_TWO}
_PARITY = {
  0: serial.PARITY_NONE,
  1: serial.PARITY_ODD,
  2: serial.PARITY_EVEN,
  3: serial.PARITY_MARK,
  4: serial.PARITY_SPACE,
}


class CeligoSerialIO:
  """The subset of ``FTDI`` that ``Celigo`` uses, backed by a pyserial port.

  ``port`` may be a COM port name or any pyserial URL (e.g. ``loop://`` in tests).
  Reads are non-blocking and return whatever bytes are available, matching the polling
  loop in ``Celigo._read_exact_bytes``.
  """

  def __init__(self, port: str, baudrate: int = 230400, write_timeout: float = 1.0) -> None:
    self.port = port
    self._baudrate = baudrate
    self._write_timeout = write_timeout
    self._ser: Optional[serial.SerialBase] = None
    self._executor: Optional[ThreadPoolExecutor] = None
    self.latency_ms_requested: Optional[int] = None

  @property
  def ser(self) -> serial.SerialBase:
    if self._ser is None:
      raise RuntimeError("Port not open. Call setup() first.")
    return self._ser

  async def _run(self, function, *args):
    if self._executor is None:
      raise RuntimeError("Port not open. Call setup() first.")
    return await asyncio.get_running_loop().run_in_executor(self._executor, function, *args)

  async def setup(self) -> None:
    self._executor = ThreadPoolExecutor(max_workers=1)
    try:
      self._ser = await self._run(
        lambda: serial.serial_for_url(
          self.port,
          baudrate=self._baudrate,
          bytesize=serial.EIGHTBITS,
          parity=serial.PARITY_NONE,
          stopbits=serial.STOPBITS_ONE,
          timeout=0,
          write_timeout=self._write_timeout,
        )
      )
    except BaseException:
      self._executor.shutdown(wait=False)
      self._executor = None
      raise
    logger.info("[CeligoSerialIO] opened %s at %d baud", self.port, self._baudrate)

  async def stop(self) -> None:
    if self._ser is not None:
      await self._run(self._ser.close)
      self._ser = None
    if self._executor is not None:
      self._executor.shutdown(wait=True)
      self._executor = None

  async def set_baudrate(self, baudrate: int) -> None:
    self.ser.baudrate = baudrate

  async def set_line_property(self, bits: int, stopbits: int, parity: int) -> None:
    self.ser.bytesize = bits
    self.ser.stopbits = _STOP_BITS[stopbits]
    self.ser.parity = _PARITY[parity]

  async def set_latency_timer(self, latency: int) -> None:
    # Not settable through a COM port; record it so callers can report the mismatch.
    self.latency_ms_requested = latency
    logger.info("[CeligoSerialIO] latency timer %d ms requested; set it in Device Manager", latency)

  async def usb_purge_rx_buffer(self) -> None:
    await self._run(self.ser.reset_input_buffer)

  async def usb_purge_tx_buffer(self) -> None:
    await self._run(self.ser.reset_output_buffer)

  async def write(self, data: bytes) -> int:
    # Logged first: once handed to the worker the bytes are sent even if the caller is
    # cancelled, and the byte log must still show them.
    logger.debug("[CeligoSerialIO] write %s", data.hex(" "))
    written = await asyncio.shield(self._run(self.ser.write, data))
    return int(written or 0)

  async def read(self, num_bytes: int = 1) -> bytes:
    data = bytes(await self._run(self.ser.read, num_bytes))
    if data:
      logger.debug("[CeligoSerialIO] read %s", data.hex(" "))
    return data
