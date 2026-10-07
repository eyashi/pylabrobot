"""Offline tests for the one-way jog, using a simulated moving motor."""

import argparse
import asyncio
import unittest
from typing import Any, Dict
from unittest.mock import patch

import jog_motor
import serial_io
import wiggle_motor
from wiggle_motor_tests import FakeMovingPort


def _args(**changes: Any) -> argparse.Namespace:
  values: Dict[str, Any] = {
    "port": "COM_TEST",
    "address": 1,
    "direction": "positive",
    "turns": 1.0,
    "ignore_bits": 0,
    "microsteps_per_tick": 64.0,  # the simulated encoder
    "stall_tolerance": 20,
    "current": 30,
    "velocity": 10240,
    "reply_timeout": 0.5,
  }
  values.update(changes)
  return argparse.Namespace(**values)


class JogTests(unittest.TestCase):
  def run_jog(self, port: FakeMovingPort, **changes: Any) -> Dict[str, Any]:
    report: Dict[str, Any] = {}
    with (
      patch.object(serial_io.serial, "serial_for_url", return_value=port),
      patch.object(wiggle_motor, "POLL_INTERVAL", 0.0),
    ):
      asyncio.run(jog_motor.run(_args(**changes), report))
    return report

  def test_moves_one_way_and_stays(self):
    port = FakeMovingPort()
    report = self.run_jog(port, turns=2.0)
    self.assertNotIn("error", report)
    self.assertEqual(port.moves, ["m30V10240P102400R"])
    self.assertEqual(port.positions[1], 102400)
    self.assertEqual(report["summary"]["turns_taken"], 2.0)

  def test_negative_direction(self):
    port = FakeMovingPort()
    self.run_jog(port, direction="negative", turns=0.5)
    self.assertEqual(port.moves, ["m30V10240D25600R"])

  def test_ignored_bit_is_logged_without_stopping(self):
    port = FakeMovingPort(flag_threshold=10000)  # bit 2 sets at 10000 microsteps
    report = self.run_jog(port, ignore_bits=4)
    self.assertFalse(report["stopped_on_flag_change"])
    self.assertEqual(port.positions[1], 51200)
    self.assertEqual([(c["from"], c["to"]) for c in report["flag_changes"]], [(3, 7)])

  def test_unignored_flag_change_stops(self):
    port = FakeMovingPort(flag_threshold=10000)
    report = self.run_jog(port)
    self.assertTrue(report["stopped_on_flag_change"])
    self.assertEqual(port.terminated, [1])
    self.assertLess(port.positions[1], 51200)

  def test_stall_stops_the_motor(self):
    port = FakeMovingPort(encoder_stall_at=20000)  # a hard stop at 20000 microsteps
    report = self.run_jog(port)
    self.assertIn("stalled", report["error"])
    self.assertIn(1, port.terminated)
    self.assertLess(port.positions[1], 51200)

  def test_refuses_a_driver_configured_motor(self):
    port = FakeMovingPort()
    port.encoder_ratio = 256000  # set by Axis._initialize()
    report = self.run_jog(port)
    self.assertIn("Power-cycle", report["error"])
    self.assertEqual(port.moves, [])

  def test_turns_are_capped(self):
    with self.assertRaises(SystemExit):
      jog_motor.main(
        ["--port", "COM_TEST", "--address", "1", "--turns", "61", "--microsteps-per-tick", "256"]
      )


if __name__ == "__main__":
  unittest.main()
