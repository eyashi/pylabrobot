"""Offline tests for the driver-path brightfield test, using a simulated controller."""

import argparse
import asyncio
import struct
import unittest
from typing import Any, Dict, List, Tuple
from unittest.mock import patch

import brightfield_test
import serial_io
from illumination_test_tests import FakeDACPort


class FakeLightingPort(FakeDACPort):
  """Adds digital-output set/clear with readback."""

  def __init__(self) -> None:
    super().__init__()
    self.digital = 0
    self.digital_writes: List[Tuple[str, int]] = []

  def _payload(self, opcode: int) -> bytes:
    if opcode in (16, 17):
      (mask,) = struct.unpack(">H", self._request_payload)
      self.digital_writes.append(("set" if opcode == 16 else "clear", mask))
      self.digital = self.digital | mask if opcode == 16 else self.digital & ~mask
      return b""
    if opcode == 34:
      return struct.pack(">H", self.digital)
    return super()._payload(opcode)


def _args(**changes: Any) -> argparse.Namespace:
  values: Dict[str, Any] = {
    "port": "COM_TEST",
    "percents": [25.0, 50.0, 70.0],
    "on_seconds": 0.0,
    "off_seconds": 0.0,
    "reply_timeout": 0.5,
  }
  values.update(changes)
  return argparse.Namespace(**values)


class BrightfieldTests(unittest.TestCase):
  def run_test(self, port: FakeLightingPort, **changes: Any) -> Dict[str, Any]:
    report: Dict[str, Any] = {}
    with patch.object(serial_io.serial, "serial_for_url", return_value=port):
      asyncio.run(brightfield_test.run(_args(**changes), report))
    return report

  def test_steps_brightfield_through_the_0_to_10_volt_mapping(self):
    port = FakeLightingPort()
    report = self.run_test(port)
    lit = [count for channel, count in port.writes if channel == 0 and count]
    self.assertEqual(lit, [1023, 2047, 2866])  # 2.5 V, 5 V, 7 V of 10 V
    self.assertEqual([step["on"]["analog_0_count"] for step in report["steps"]], lit)
    self.assertEqual(report["final_outputs"], {"analog_0_count": 0, "digital_outputs": "0x0000"})

  def test_only_analog_output_0_lights_and_no_digital_line_is_set(self):
    port = FakeLightingPort()
    self.run_test(port)
    self.assertEqual({channel for channel, count in port.writes if count}, {0})
    self.assertEqual({kind for kind, _mask in port.digital_writes}, {"clear"})
    self.assertEqual({mask for _kind, mask in port.digital_writes}, {1 << 6})

  def test_interruption_turns_illumination_off(self):
    port = FakeLightingPort()

    real_outputs = brightfield_test._outputs
    calls: List[int] = []

    async def interrupt(celigo: Any) -> Dict[str, Any]:
      calls.append(1)
      if len(calls) == 2:  # the read just after brightfield turns on
        raise asyncio.CancelledError
      return await real_outputs(celigo)

    report: Dict[str, Any] = {}
    with (
      patch.object(serial_io.serial, "serial_for_url", return_value=port),
      patch.object(brightfield_test, "_outputs", interrupt),
      self.assertRaises(asyncio.CancelledError),
    ):
      asyncio.run(brightfield_test.run(_args(), report))
    self.assertIn((0, 1023), port.writes)
    self.assertEqual(port.dac[0], 0)

  def test_intensity_is_capped(self):
    with self.assertRaises(SystemExit):
      brightfield_test.main(["--port", "COM_TEST", "--percents", "90"])


if __name__ == "__main__":
  unittest.main()
