"""Offline tests for the galvo exercise, using a simulated galvo controller."""

import argparse
import asyncio
import struct
import unittest
from typing import Any, Dict, List, Tuple
from unittest.mock import patch

import galvo_test
import serial_io
from io_survey_tests import FakeIOPort

from pylabrobot.revvity.celigo.galvo import dac_count_to_volts

# Galvo opcodes this test may send: MOVE, STATUS, CALIBRATE, CAL_DATA, SET_WINDOW.
GALVO_OPCODES = {7, 12, 27, 28, 29}


class FakeGalvoPort(FakeIOPort):
  def __init__(self, calibration_status: int = 0) -> None:
    super().__init__()
    self.calibration_status = calibration_status
    self.dac = {0: 32768, 1: 32768}
    self.calibrated = {0: False, 1: False}
    self.windows: List[Tuple[int, int, int]] = []
    self.moves: List[Tuple[int, int]] = []

  def _payload(self, opcode: int) -> bytes:
    payload = self._request_payload
    if opcode == 29:
      self.windows.append(struct.unpack(">HHH", payload))
      return b""
    if opcode == 27:
      index, _timeout, _flag = struct.unpack(">HHH", payload)
      self.calibrated[index] = self.calibration_status == 0
      return struct.pack(">H", self.calibration_status)
    if opcode == 7:
      index, dac, _wait, _timeout = struct.unpack(">HiHH", payload)
      self.moves.append((index, dac))
      self.dac[index] = dac
      return struct.pack(">H", 0)
    if opcode == 12:
      return struct.pack(
        ">BBHHiiiBhh",
        int(self.calibrated[0]),
        int(self.calibrated[1]),
        self.dac[0],
        self.dac[1],
        0,
        0,
        0,
        0,
        0,
        0,
      )
    if opcode == 28:
      (index,) = struct.unpack(">H", payload)
      pairs = [(10, -10), (12, -8)] if self.calibrated[index] else []
      return struct.pack(">h", len(pairs)) + b"".join(struct.pack(">hh", *p) for p in pairs)
    return super()._payload(opcode)


def _args(**changes: Any) -> argparse.Namespace:
  values: Dict[str, Any] = {
    "port": "COM_TEST",
    "through": "calibrate",
    "step": 0.5,
    "max_volts": 1.0,
    "pause": 0.0,
    "reply_timeout": 0.5,
  }
  values.update(changes)
  return argparse.Namespace(**values)


class GalvoTestTests(unittest.TestCase):
  def run_test(self, port: FakeGalvoPort, **changes: Any) -> Dict[str, Any]:
    report: Dict[str, Any] = {}
    with patch.object(serial_io.serial, "serial_for_url", return_value=port):
      asyncio.run(galvo_test.run(_args(**changes), report))
    return report

  def test_calibrate_follows_the_driver_sequence(self):
    port = FakeGalvoPort()
    report = self.run_test(port)
    self.assertTrue(report["calibrate"]["passed"])
    self.assertEqual(port.windows, [(0, 20, 20), (1, 20, 20)])
    self.assertEqual(len(report["calibrate"]["x_sweeps"]), 2)
    self.assertEqual(report["calibrate"]["y_errors_after"]["count"], 2)
    self.assertEqual(port.moves, [])
    self.assertLessEqual(set(port.opcodes) - {3, 9, 22, 23}, GALVO_OPCODES)

  def test_moves_step_out_and_back_to_zero(self):
    port = FakeGalvoPort()
    report = self.run_test(port, through="move")
    volts = [(index, round(dac_count_to_volts(dac), 3)) for index, dac in port.moves]
    self.assertEqual(volts, [(0, 0.5), (0, -0.5), (0, 0.0), (1, 0.5), (1, -0.5), (1, 0.0)])
    self.assertTrue(all(move["outcome"] == "settled" for move in report["moves"]))

  def test_failed_calibration_skips_moves(self):
    port = FakeGalvoPort(calibration_status=1)
    report = self.run_test(port, through="move")
    self.assertFalse(report["calibrate"]["passed"])
    self.assertIn("skipping moves", report["error"])
    self.assertEqual(port.moves, [])

  def test_step_is_limited(self):
    for argv in (["--step", "1.5"], ["--max-volts", "3"]):
      with self.subTest(argv=argv), self.assertRaises(SystemExit):
        galvo_test.main(["--port", "COM_TEST", *argv])


if __name__ == "__main__":
  unittest.main()
