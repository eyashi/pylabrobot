"""Offline tests for the door demonstration."""

import asyncio
import unittest
from typing import List
from unittest.mock import patch

import door_demo
import serial_io
from home_stepwise_tests import FakeStagePort
from serial_io import CeligoSerialIO

from pylabrobot.revvity.celigo import CeligoError


class _Axis:
  async def request_position(self) -> float:
    return 0.0


class RecordingCeligo:
  """Stands in for Celigo, recording the driver calls the demo makes."""

  def __init__(self, fail_on: str = "") -> None:
    self.calls: List[str] = []
    self.fail_on = fail_on
    self.x_axis = self.y_axis = self.z_axis = _Axis()

  async def _call(self, name: str) -> None:
    self.calls.append(name)
    if name == self.fail_on:
      raise CeligoError(f"{name} failed")

  async def setup(self, home: bool = True) -> None:
    await self._call("setup" if home else "setup(home=False)")

  async def open_drawer(self) -> None:
    await self._call("open_drawer")

  async def close_drawer_to_sample_mm(self, x: float, y: float) -> None:
    await self._call(f"close_drawer_to_sample_mm({x:g}, {y:g})")

  async def stop(self) -> None:
    await self._call("stop")


class DoorDemoTests(unittest.TestCase):
  def test_runs_the_driver_flow_in_order(self):
    celigo = RecordingCeligo()
    prompts: List[str] = []
    asyncio.run(door_demo.run_demo(celigo, None, prompt=prompts.append))  # type: ignore[arg-type]
    self.assertEqual(
      celigo.calls, ["setup", "open_drawer", "close_drawer_to_sample_mm(0, 0)", "stop"]
    )
    self.assertEqual(len(prompts), 1)

  def test_stop_runs_when_a_step_fails(self):
    celigo = RecordingCeligo(fail_on="open_drawer")
    with self.assertRaises(CeligoError):
      asyncio.run(door_demo.run_demo(celigo, 0.0))  # type: ignore[arg-type]
    self.assertEqual(celigo.calls, ["setup", "open_drawer", "stop"])

  def test_refuses_to_start_with_the_stage_out_of_the_door(self):
    port = FakeStagePort()
    port.raw = port.target = port.encoder = float(FakeStagePort.FAR_AT)  # Y at the front
    with patch.object(serial_io.serial, "serial_for_url", return_value=port):
      with self.assertRaisesRegex(CeligoError, "out through the door"):
        asyncio.run(door_demo.ensure_stage_inside("COM_TEST"))
    port.raw = port.target = port.encoder = 0.0  # inside
    with patch.object(serial_io.serial, "serial_for_url", return_value=port):
      asyncio.run(door_demo.ensure_stage_inside("COM_TEST"))

  def test_builds_the_driver_on_the_legacy_config_without_a_camera(self):
    with patch("pylabrobot.revvity.celigo.celigo.FTDI"):
      celigo = door_demo.make_demo_celigo("COM_TEST")
    self.assertIsNone(celigo.camera)
    self.assertIsInstance(celigo.io, CeligoSerialIO)
    self.assertEqual(celigo.move_timeout, door_demo.MOVE_TIMEOUT)
    self.assertEqual(
      [axis.axis_index for axis in (celigo.x_axis, celigo.y_axis, celigo.z_axis)], [1, 2, 3]
    )
    self.assertEqual(celigo.dichroic_filter.axis_index, 4)


if __name__ == "__main__":
  unittest.main()
