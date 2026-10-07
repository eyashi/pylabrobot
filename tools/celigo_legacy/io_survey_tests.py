"""Offline tests for the read-only I/O survey, using a simulated controller."""

import argparse
import asyncio
import struct
import unittest
from typing import Any, Dict, List
from unittest.mock import patch

import io_survey
import serial_io
from probe_motors_tests import FakeMotorPort

# Every controller opcode the survey may send: ABORT and the handshake reads, then
# digital/analog reads, galvo status/cal/trace reads, signal diagnostics, barcode read, and
# the motor-query tunnel.
READ_ONLY_OPCODES = {3, 9, 12, 15, 20, 22, 23, 28, 31, 34, 35, 43, 46, 47}


class FakeIOPort(FakeMotorPort):
  def __init__(self) -> None:
    super().__init__()
    self.digital_inputs = 0x0001
    # A switch that toggles input bit 3 after this many digital-input reads.
    self.toggle_after_reads = None
    self._digital_reads = 0
    self.diagnostic_operations: List[int] = []
    self.unsupported_opcodes: set = set()

  def write(self, data: bytes) -> int:
    self._request_payload = data[11:]
    return super().write(data)

  def _payload(self, opcode: int) -> bytes:
    payload = self._request_payload
    if opcode in self.unsupported_opcodes:
      return b""  # too short to decode, as from hardware this unit lacks
    if opcode == 15:
      self._digital_reads += 1
      if self.toggle_after_reads is not None and self._digital_reads > self.toggle_after_reads:
        return struct.pack(">H", self.digital_inputs | 0x0008)
      return struct.pack(">H", self.digital_inputs)
    if opcode == 34:
      return struct.pack(">H", 0x0002)
    if opcode == 35:
      (channel,) = struct.unpack(">H", payload)
      return struct.pack(">HH", channel, 0)
    if opcode == 20:
      return struct.pack(">H", 2048)
    if opcode == 12:
      return struct.pack(">BBHHiiiBhh", 1, 1, 32768, 32768, 0, 0, 0, 0, 0, 0)
    if opcode == 28:
      return struct.pack(">h", 0)
    if opcode == 31:
      return struct.pack(">H", 0)
    if opcode == 43:
      (operation,) = struct.unpack(">h", payload)
      self.diagnostic_operations.append(operation)
      return struct.pack(">i", 1)
    if opcode == 46:
      return struct.pack(">HH", 0, 4) + b"ABCD"
    return super()._payload(opcode)


def _args(**changes: Any) -> argparse.Namespace:
  values: Dict[str, Any] = {
    "port": "COM_TEST",
    "mode": "snapshot",
    "seconds": 0.5,
    "interval": 0.0,
    "analog_threshold": 20,
    "reply_timeout": 0.5,
  }
  values.update(changes)
  return argparse.Namespace(**values)


class IOSurveyTests(unittest.TestCase):
  def run_survey(self, port: FakeIOPort, **changes: Any) -> Dict[str, Any]:
    report: Dict[str, Any] = {}
    with patch.object(serial_io.serial, "serial_for_url", return_value=port):
      asyncio.run(io_survey.run(_args(**changes), report))
    return report

  def test_snapshot_reads_every_signal(self):
    port = FakeIOPort()
    snapshot = self.run_survey(port)["snapshot"]
    self.assertEqual(snapshot["digital_inputs"], {"mask": "0x0001", "set_bits": [0]})
    self.assertEqual(snapshot["digital_outputs"]["set_bits"], [1])
    self.assertEqual(snapshot["analog_input_2"], 2048)
    self.assertFalse(snapshot["galvo_status"]["x_busy"])
    self.assertEqual(snapshot["barcode_buffer"], "ABCD")
    self.assertEqual(snapshot["motor_1_input_flags"], 4)

  def test_sends_only_reads(self):
    port = FakeIOPort()
    self.run_survey(port)
    self.run_survey(port, mode="watch")
    self.assertLessEqual(set(port.opcodes), READ_ONLY_OPCODES)
    # Only the read sub-commands of SIGNAL_DIAGNOSTICS; 1-3 would drive the trigger line.
    self.assertEqual(set(port.diagnostic_operations), {4, 5, 6})
    for tokens in port.motor_commands:
      self.assertEqual(tokens, "?4")

  def test_unsupported_hardware_is_recorded_not_fatal(self):
    port = FakeIOPort()
    port.unsupported_opcodes = {12}  # no galvo controller
    snapshot = self.run_survey(port)["snapshot"]
    self.assertTrue(snapshot["galvo_status"].startswith("FAILED"))
    self.assertEqual(snapshot["analog_input_0"], 2048)

  def test_watch_reports_a_toggled_input_bit(self):
    port = FakeIOPort()
    port.toggle_after_reads = 3
    changes = self.run_survey(port, mode="watch")["changes"]
    toggles = [change for change in changes if change.get("signal") == "digital_inputs"]
    self.assertEqual(toggles[0]["bits_toggled"], [3])


if __name__ == "__main__":
  unittest.main()
