"""Offline tests for the analog-output illumination test, using a simulated controller."""

import argparse
import asyncio
import struct
import unittest
from typing import Any, Dict, List, Tuple
from unittest.mock import patch

import illumination_test
import serial_io
from io_survey_tests import FakeIOPort

# Opcodes this test may send: handshake (ABORT, STATUS, SEND_CONFIG, MOTOR_CONFIG), digital
# output READBACK, analog output WRITE and READBACK, and analog input READ.
ALLOWED_OPCODES = {3, 9, 22, 23, 34, 18, 35, 20}


class FakeDACPort(FakeIOPort):
  def __init__(self, fail_on_write_number: int = 0) -> None:
    super().__init__()
    self.dac = {channel: 0 for channel in range(4)}
    self.writes: List[Tuple[int, int]] = []
    self.fail_on_write_number = fail_on_write_number

  def _payload(self, opcode: int) -> bytes:
    payload = self._request_payload
    if opcode == 18:
      channel, count = struct.unpack(">HH", payload)
      self.writes.append((channel, count))
      if self.fail_on_write_number and len(self.writes) == self.fail_on_write_number:
        return b"\x00"  # an unexpected reply payload does not matter for a write
      self.dac[channel] = count
      return b""
    if opcode == 35:
      (channel,) = struct.unpack(">H", payload)
      return struct.pack(">HH", channel, self.dac[channel])
    return super()._payload(opcode)


def _args(**changes: Any) -> argparse.Namespace:
  values: Dict[str, Any] = {
    "port": "COM_TEST",
    "channels": [0, 1, 2, 3],
    "count": 410,
    "on_seconds": 0.0,
    "off_seconds": 0.0,
    "reply_timeout": 0.5,
  }
  values.update(changes)
  return argparse.Namespace(**values)


class IlluminationTests(unittest.TestCase):
  def run_test(self, port: FakeDACPort, **changes: Any) -> Dict[str, Any]:
    report: Dict[str, Any] = {}
    with patch.object(serial_io.serial, "serial_for_url", return_value=port):
      asyncio.run(illumination_test.run(_args(**changes), report))
    return report

  def test_lights_each_channel_once_then_zeroes_it(self):
    port = FakeDACPort()
    report = self.run_test(port)
    lit = [write for write in port.writes if write[1] != 0]
    self.assertEqual(lit, [(0, 410), (1, 410), (2, 410), (3, 410)])
    self.assertEqual([entry["readback_on"] for entry in report["channels"]], [410] * 4)
    self.assertEqual(port.dac, {0: 0, 1: 0, 2: 0, 3: 0})
    self.assertEqual(report["final_zeroing"], {0: 0, 1: 0, 2: 0, 3: 0})

  def test_only_one_channel_is_ever_on(self):
    port = FakeDACPort()
    state = {channel: 0 for channel in range(4)}
    self.run_test(port)
    for channel, count in port.writes:
      state[channel] = count
      self.assertLessEqual(sum(1 for value in state.values() if value), 1)

  def test_digital_outputs_are_never_written(self):
    port = FakeDACPort()
    self.run_test(port)
    self.assertLessEqual(set(port.opcodes), ALLOWED_OPCODES)

  def test_interruption_still_zeroes_every_channel(self):
    port = FakeDACPort()

    async def interrupt(_celigo: Any) -> None:
      raise asyncio.CancelledError  # e.g. Ctrl+C while output 0 is lit

    report: Dict[str, Any] = {}
    with (
      patch.object(serial_io.serial, "serial_for_url", return_value=port),
      patch.object(illumination_test, "_analog_inputs", interrupt),
      self.assertRaises(asyncio.CancelledError),
    ):
      asyncio.run(illumination_test.run(_args(), report))
    self.assertIn((0, 410), port.writes)
    self.assertEqual(port.dac, {0: 0, 1: 0, 2: 0, 3: 0})
    self.assertEqual(report["final_zeroing"], {0: 0, 1: 0, 2: 0, 3: 0})

  def test_count_is_capped(self):
    with self.assertRaises(SystemExit):
      illumination_test.main(["--port", "COM_TEST", "--count", "1025"])


if __name__ == "__main__":
  unittest.main()
