"""Offline tests for the wheel index check, using the simulated axis on motor 4."""

import argparse
import asyncio
import unittest
from typing import Any, Dict
from unittest.mock import patch

import home_stepwise
import serial_io
import wheel_index_test
from home_stepwise_tests import FakeStagePort, _no_sleep


class FakeWheelPort(FakeStagePort):
  ADDRESS = 4


def _args(**changes: Any) -> argparse.Namespace:
  values: Dict[str, Any] = {
    "port": "COM_TEST",
    "max_travel": 10500,
    "map": False,
    "reply_timeout": 0.5,
  }
  values.update(changes)
  return argparse.Namespace(**values)


class WheelIndexTests(unittest.TestCase):
  def run_test(self, port: FakeWheelPort) -> Dict[str, Any]:
    report: Dict[str, Any] = {}
    with (
      patch.object(serial_io.serial, "serial_for_url", return_value=port),
      patch.object(home_stepwise, "POLL_INTERVAL", 0.0),
      patch("pylabrobot.revvity.celigo.motion.asyncio.sleep", _no_sleep),
    ):
      asyncio.run(wheel_index_test.run(_args(), report))
    return report

  def test_an_index_stops_the_wheel_at_zero(self):
    report = self.run_test(FakeWheelPort())
    self.assertTrue(report["has_index"])
    self.assertEqual(report["search"]["stopped_by"], "motor")
    self.assertEqual(report["encoder_ratio_readback"], 25.6)

  def test_no_index_is_stopped_by_the_travel_limit(self):
    report = self.run_test(FakeWheelPort(has_index=False, z_ignores_distance=True))
    self.assertFalse(report["has_index"])
    self.assertIn("travelled", report["search"]["stopped_by"])
    self.assertLessEqual(report["search"]["travelled_ticks"], 10500 + 100)


if __name__ == "__main__":
  unittest.main()
