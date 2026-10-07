"""Offline tests for the stepwise homing, using a simulated stage axis on motor 2."""

import argparse
import asyncio
import re
import struct
import unittest
from typing import Any, Dict, List
from unittest.mock import patch

import home_stepwise
import serial_io
from probe_controller_tests import FakeControllerPort
from probe_motors_tests import _oem_reply

_PREFIX = r"(?<![a-zA-Z])"


class FakeStagePort(FakeControllerPort):
  """Motor 2 as a stage: home sensor at raw tick -1000, encoder index every 200 ticks.

  Positions are encoder ticks once ``aE`` is configured, unless ``units_are_ticks`` is
  False, in which case moves and ``?0`` stay in microsteps (256 per tick).
  """

  SENSOR_AT = -1000
  INDEX_PHASE = 37

  def __init__(
    self,
    honor_limits: bool = True,
    has_index: bool = True,
    units_are_ticks: bool = True,
    slack_ticks: float = 0.0,
  ) -> None:
    super().__init__(firmware=0x010300)
    self.honor_limits = honor_limits
    self.has_index = has_index
    self.units_are_ticks = units_are_ticks
    self.raw = -500.0  # physical position in ticks
    # The encoder trails the commanded position by up to ``slack_ticks`` after a reversal.
    self.slack_ticks = slack_ticks
    self.encoder = self.raw
    self.target = self.raw
    self.offset = 0.0  # set by an index home
    self.mode = 0
    self.encoder_ratio = 1000
    self.zero_at_target = False
    self.executed: List[str] = []

  def _sensor(self) -> bool:
    return self.raw <= self.SENSOR_AT

  def _units_per_tick(self) -> float:
    return 1.0 if self.units_are_ticks else 256.0

  def _step(self) -> None:
    remaining = self.target - self.raw
    if not remaining:
      return
    if self.mode & 2 and self.honor_limits and remaining < 0 and self._sensor():
      self.target = self.raw  # limit mode stops at the home sensor
      return
    self.raw += max(-5.0, min(5.0, remaining))
    self.encoder = min(max(self.encoder, self.raw - self.slack_ticks), self.raw + self.slack_ticks)
    if self.raw == self.target and self.zero_at_target:
      self.offset = self.raw
      self.encoder = self.raw
      self.zero_at_target = False

  def _execute(self, tokens: str) -> None:
    self.executed.append(tokens)
    scale = 1.0 / self._units_per_tick()
    for match in re.finditer(_PREFIX + r"n(\d+)", tokens):
      self.mode = int(match.group(1))
    if match := re.search(_PREFIX + r"aE(\d+)", tokens):
      self.encoder_ratio = int(match.group(1))
    if match := re.search(_PREFIX + r"([PD])(\d+)R$", tokens):
      sign = 1 if match.group(1) == "P" else -1
      self.target = self.raw + sign * int(match.group(2)) * scale
    elif match := re.search(_PREFIX + r"A(-?\d+)R$", tokens):
      self.target = self.offset + int(match.group(1)) * scale
    elif match := re.search(_PREFIX + r"Z(\d+)R$", tokens):
      # As observed on the instrument: zero the counters, drive back into the home sensor,
      # then stop and zero again at the first encoder index past the sensor edge.
      distance = int(match.group(1)) * scale
      start = self.raw
      self.offset = self.encoder = start
      edge = min(start, float(self.SENSOR_AT))
      index = edge - ((edge - self.INDEX_PHASE) % 200)
      if self.has_index and start - index <= distance:
        self.target, self.zero_at_target = index, True
      else:
        self.target = start - distance

  def _payload(self, opcode: int) -> bytes:
    if opcode != 47:
      return super()._payload(opcode)
    request = self._last_request
    address = int(chr(request[1]))
    tokens = request[3:-2].decode("ascii")
    status, text = 0x60, ""
    if address != 2:
      text = "EZStepper AllMotion V7.21" if tokens == "&" else "0"
    elif tokens == "&":
      text = "EZStepper AllMotion V7.21 3-9-10"
    elif tokens == "T":
      self.target = self.raw
    elif tokens == "Q":
      self._step()
      if self.target != self.raw:
        status = 0x40
    elif tokens == "?0":
      text = str(round((self.raw - self.offset) * self._units_per_tick()))
    elif tokens == "?8":
      text = str(round(self.encoder - self.offset))
    elif tokens == "?4":
      text = str(7 if self._sensor() else 3)
    elif tokens == "?aE":
      text = str(self.encoder_ratio)
    elif tokens.endswith("R"):
      self._execute(tokens)
    reply = _oem_reply(text, status)
    return struct.pack(">HH", 0, len(reply)) + reply

  def write(self, data: bytes) -> int:
    self._last_request = data[11:]
    return super().write(data)


def _args(**changes: Any) -> argparse.Namespace:
  values: Dict[str, Any] = {
    "port": "COM_TEST",
    "axis": "y",
    "home_type": "Normal_Accurate",
    "through": "zero",
    "driver_home": False,
    "units_per_tick": None,
    "overtravel": 30,
    "stall_tolerance": 30,
    "reply_timeout": 0.5,
  }
  values.update(changes)
  return argparse.Namespace(**values)


_real_sleep = asyncio.sleep


async def _no_sleep(_seconds: float) -> None:
  # Driver polling delays are skipped; patching asyncio.sleep patches it everywhere.
  await _real_sleep(0)


class HomeStepwiseTests(unittest.TestCase):
  def run_home(self, port: FakeStagePort, **changes: Any) -> Dict[str, Any]:
    report: Dict[str, Any] = {}
    with (
      patch.object(serial_io.serial, "serial_for_url", return_value=port),
      patch.object(home_stepwise, "POLL_INTERVAL", 0.0),
      patch("pylabrobot.revvity.celigo.motion.asyncio.sleep", _no_sleep),
    ):
      asyncio.run(home_stepwise.run(_args(**changes), report))
    return report

  def outcomes(self, report: Dict[str, Any]) -> Dict[str, str]:
    return {name: stage["outcome"] for name, stage in report["stages"].items()}

  def test_all_stages_home_to_the_index(self):
    port = FakeStagePort()
    report = self.run_home(port)
    self.assertLess(port.raw, FakeStagePort.SENSOR_AT)  # the index lies inside the sensor
    self.assertNotIn("error", report)
    self.assertEqual(set(self.outcomes(report).values()), {"ok"})
    self.assertEqual(report["stages"]["units"]["verdict"], "encoder ticks (matches the driver)")
    self.assertEqual(report["stages"]["zero"]["position_mm"], 0.0)
    self.assertEqual(port.offset % 200, FakeStagePort.INDEX_PHASE % 200)

  def test_slack_does_not_read_as_a_stall(self):
    port = FakeStagePort(slack_ticks=1.0)
    port.raw = port.target = port.encoder = -100.0  # a 900-tick seek, as on the instrument
    report = self.run_home(port, through="seek")
    self.assertEqual(report["stages"]["units"]["encoder_ticks_moved"], 19)
    self.assertEqual(report["stages"]["units"]["units_per_tick"], 1.0)
    self.assertEqual(report["stages"]["seek"]["outcome"], "ok")

  def test_starting_deep_in_the_sensor_clears_it_first(self):
    port = FakeStagePort()
    port.raw = port.target = port.encoder = -1300.0  # 300 ticks inside the sensor
    report = self.run_home(port, through="seek")
    cleared = report["stages"]["seek"]["cleared_sensor_first"]
    self.assertAlmostEqual(cleared["sensor_depth_ticks"], 300, delta=25)
    self.assertEqual(report["stages"]["seek"]["outcome"], "ok")
    self.assertAlmostEqual(port.raw, FakeStagePort.SENSOR_AT, delta=25)  # back at the edge

  def test_init_only_sends_no_motion(self):
    port = FakeStagePort()
    report = self.run_home(port, through="init")
    self.assertEqual(self.outcomes(report), {"init": "ok"})
    self.assertEqual(port.raw, -500.0)
    self.assertEqual(port.encoder_ratio, 256000)
    for tokens in port.executed:
      self.assertIsNone(re.search(_PREFIX + r"[PDAZ]-?\d+R$", tokens), tokens)

  def test_microstep_units_stop_before_the_seek(self):
    port = FakeStagePort(units_are_ticks=False)
    report = self.run_home(port)
    self.assertIn("microsteps", report["stages"]["units"]["outcome"])
    self.assertNotIn("seek", report["stages"])

  def test_limit_mode_that_does_not_stop_trips_the_watchdog(self):
    port = FakeStagePort(honor_limits=False)
    report = self.run_home(port)
    self.assertIn("past the home-sensor edge", report["stages"]["seek"]["outcome"])
    self.assertNotIn("backoff", report["stages"])
    self.assertGreater(port.raw, FakeStagePort.SENSOR_AT - 30 - 100)
    self.assertEqual(port.target, port.raw)  # terminated

  def test_missing_index_stops_on_reentering_the_sensor(self):
    port = FakeStagePort(has_index=False)
    report = self.run_home(port)
    self.assertIn("no encoder index found", report["stages"]["index"]["outcome"])
    self.assertNotIn("zero", report["stages"])

  def test_driver_home_matches_the_stepwise_result(self):
    port = FakeStagePort()
    report = self.run_home(port, driver_home=True, units_per_tick=1.0)
    self.assertEqual(self.outcomes(report), {"init": "ok", "driver_home": "ok"})
    self.assertEqual(report["stages"]["driver_home"]["position_mm"], 0.0)

  def test_guarded_never_cancels_the_watchdog_mid_query(self):
    interrupted: List[bool] = []

    class SlowQueryWatchdog:
      axis = None

      async def run(self, started_at: float, stop: asyncio.Event) -> None:
        while not stop.is_set():
          try:
            await _real_sleep(0.02)  # a query in flight
          except asyncio.CancelledError:
            interrupted.append(True)
            raise

    async def quick_operation() -> str:
      await _real_sleep(0.005)
      return "done"

    result = asyncio.run(
      home_stepwise.guarded(quick_operation(), SlowQueryWatchdog(), 0.0)  # type: ignore[arg-type]
    )
    self.assertEqual(result, "done")
    self.assertEqual(interrupted, [])

  def test_driver_home_requires_measured_units(self):
    with self.assertRaises(SystemExit):
      home_stepwise.main(["--port", "COM_TEST", "--driver-home"])


if __name__ == "__main__":
  unittest.main()
