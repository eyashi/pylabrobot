"""Offline tests for the camera bring-up stages, using a simulated Vimba library."""

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import camera_probe

from pylabrobot.revvity.celigo.allied_vision_camera import AlliedVisionCamera
from pylabrobot.revvity.celigo.camera import CameraFrame
from pylabrobot.revvity.celigo.tests.allied_vision_camera_tests import _FakeVimbaLibrary


def _frame(values: bytes, bit_depth: int = 8) -> CameraFrame:
  return CameraFrame(values, len(values), 1, bit_depth, 1.0, 0.0, 0.0)


class TestUnusedLowBits(unittest.TestCase):
  def test_counts_bits_zero_in_every_pixel(self):
    self.assertEqual(camera_probe.unused_low_bits(_frame(bytes([4, 8, 12]))), 2)
    self.assertEqual(camera_probe.unused_low_bits(_frame(bytes([3, 8]))), 0)
    self.assertEqual(camera_probe.unused_low_bits(_frame(bytes([0, 0]))), 8)


class TestStages(unittest.TestCase):
  def _run(self, *argv: str) -> dict:
    library = _FakeVimbaLibrary()
    with (
      tempfile.TemporaryDirectory() as out,
      patch.object(
        camera_probe,
        "make_camera",
        lambda args: AlliedVisionCamera(
          library=library, pixel_format=args.pixel_format, discovery_timeout=0
        ),
      ),
    ):
      code = camera_probe.main([*argv, "--out", out])
      reports = list(Path(out).glob("*.json"))
      self.assertEqual(len(reports), 1)
      report = json.loads(reports[0].read_text(encoding="utf-8"))
      report["exit_code"] = code
      report["pngs"] = sorted(path.name for path in Path(out).glob("*.png"))
    self.assertIsNone(library.open_handle)
    return report

  def test_info_reports_identity_and_features(self):
    report = self._run("info")
    self.assertEqual(report["exit_code"], 0)
    self.assertEqual(report["camera"]["model"], "Pike F-505B")
    self.assertEqual(report["exposure_feature"], "ExposureTime")
    self.assertEqual(report["features"], [])

  def test_capture_records_each_exposure(self):
    report = self._run("capture", "--exposures-ms", "1", "5", "--gain", "2")
    self.assertEqual(report["exit_code"], 0)
    self.assertEqual(report["gain_applied"], 2.0)
    self.assertEqual([f["applied_exposure_ms"] for f in report["frames"]], [1.0, 5.0])
    self.assertEqual(report["frames"][0]["size"], [8, 6, 16])
    self.assertEqual(len(report["pngs"]), 2)

  def test_roi_captures_in_the_requested_window(self):
    report = self._run("roi", "--width", "4", "--height", "2", "--pixel-format", "Mono8")
    self.assertEqual(report["exit_code"], 0)
    self.assertEqual(report["full_frame"], [8, 6])
    self.assertEqual(report["frame"]["size"], [4, 2, 8])

  def test_errors_are_reported_with_a_failing_exit_code(self):
    report = self._run("roi", "--width", "3", "--height", "2")
    self.assertEqual(report["exit_code"], 1)
    self.assertIn("not a multiple", report["error"])


if __name__ == "__main__":
  unittest.main()
