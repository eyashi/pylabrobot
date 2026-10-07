"""Offline tests for the legacy Celigo probe, using a simulated controller port."""

import asyncio
import struct
import unittest
from typing import List, Optional
from unittest.mock import patch

import probe_controller
import serial_io
from controller import connect, disconnect, make_celigo

from pylabrobot.revvity.celigo.celigo import _fletcher16


def _response(opcode: int, sequence: int, payload: bytes = b"", ack: int = 0) -> bytes:
  header = bytearray(12)
  header[0] = ack
  header[1] = opcode
  struct.pack_into(">ii", header, 2, sequence, len(payload))
  header[10], header[11] = _fletcher16(header, 10)
  return bytes(header) + payload


class FakeControllerPort:
  """A pyserial-like port that answers the read-only handshake commands."""

  def __init__(self, firmware: int = 0x010200, silent: bool = False) -> None:
    self.firmware = firmware
    self.silent = silent
    self.opcodes: List[int] = []
    self._pending = bytearray()
    self.baudrate: Optional[int] = None
    self.bytesize = self.stopbits = self.parity = None

  def write(self, data: bytes) -> int:
    opcode = data[0]
    sequence, total_length = struct.unpack_from(">ii", data, 1)
    assert total_length == len(data)
    assert (data[9], data[10]) == _fletcher16(data, 9)
    self.opcodes.append(opcode)
    if not self.silent:
      self._pending += _response(opcode, sequence, self._payload(opcode))
    return len(data)

  def _payload(self, opcode: int) -> bytes:
    if opcode == 23:  # CONTROLLER_STATUS: interlock open
      return struct.pack(">II", 4, 0)
    if opcode == 22:  # SEND_CONFIG
      return struct.pack(">hii", 0, self.firmware, 64)
    if opcode == 9:  # SEND_MOTOR_CONFIG: motors 1 and 2 on UART 0, motor 3 on UART 1
      uarts = [bytes([0, 1, 2, 127, 127]), bytes([0, 3, 127, 127, 127])]
      uarts += [bytes([0, 127, 127, 127, 127])] * 6
      return b"".join(uarts)
    if opcode == 15:  # READ_DIG_PORT
      return struct.pack(">H", 0x0005)
    return b""

  def read(self, num_bytes: int) -> bytes:
    data = bytes(self._pending[:num_bytes])
    del self._pending[:num_bytes]
    return data

  def reset_input_buffer(self) -> None:
    self._pending.clear()

  def reset_output_buffer(self) -> None:
    pass

  def close(self) -> None:
    pass


class ProbeTests(unittest.TestCase):
  def run_probe(self, port: FakeControllerPort, reply_timeout: float = 0.5) -> dict:
    with patch.object(serial_io.serial, "serial_for_url", return_value=port):
      return asyncio.run(probe_controller.probe("COM_TEST", reply_timeout))

  def test_reports_legacy_controller(self):
    port = FakeControllerPort(firmware=0x010200)
    report = self.run_probe(port)
    self.assertEqual(report["handshake"], "ok")
    self.assertEqual(report["controller_info"]["firmware_version"], (1, 2, 0))
    self.assertIn("legacy", report["motor_command_framing"])
    self.assertTrue(report["controller_status"]["interlock_open"])
    self.assertEqual(
      report["detected_motors"],
      [
        {"uart_index": 0, "motor_index": 1},
        {"uart_index": 0, "motor_index": 2},
        {"uart_index": 1, "motor_index": 3},
      ],
    )
    self.assertEqual(report["digital_inputs"], "0x0005")

  def test_sends_only_read_only_opcodes(self):
    port = FakeControllerPort()
    self.run_probe(port)
    # ABORT, CONTROLLER_STATUS, SEND_CONFIG, SEND_MOTOR_CONFIG, READ_DIG_PORT
    self.assertLessEqual(set(port.opcodes), {3, 23, 22, 9, 15})

  def test_newer_firmware_uses_length_prefixed_framing(self):
    report = self.run_probe(FakeControllerPort(firmware=0x010300))
    self.assertIn("opcode 47", report["motor_command_framing"])

  def test_silent_controller_fails_handshake(self):
    report = self.run_probe(FakeControllerPort(silent=True), reply_timeout=0.05)
    self.assertTrue(report["handshake"].startswith("FAILED"))

  def test_connect_applies_serial_line_settings(self):
    port = FakeControllerPort()

    async def run() -> None:
      celigo = make_celigo("COM_TEST")
      await connect(celigo)
      await disconnect(celigo)

    with patch.object(serial_io.serial, "serial_for_url", return_value=port):
      asyncio.run(run())
    self.assertEqual(
      (port.baudrate, port.bytesize, port.stopbits, port.parity), (230400, 8, 1, "N")
    )


if __name__ == "__main__":
  unittest.main()
