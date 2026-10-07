"""Run the Celigo driver's linear-axis homing one stage at a time, under a watchdog.

``LinearAxis.home()`` (pylabrobot/revvity/celigo/motion.py) runs as one call. This script
issues the same driver calls in order and stops after ``--through``, so each stage can be
checked on hardware before the next one is attempted:

  init     Axis._initialize(): replay the draft config into the motor (no motion)
  units    move +20 / -20 with the homing move routine; reports whether positions are now
           encoder ticks (the driver's assumption) or microsteps
  seek     limit mode on, drive negative up to 25000 units; the MOTOR must stop itself
           at the home sensor
  backoff  move +HomingShortMove; the sensor must clear
  index    the driver's Z search toward home with limits disabled. Normal/Normal_Accurate
           zero at the next encoder index; NormalWithHardstopCheck (N1, no index) must stop
           and zero at the home-sensor edge
  zero     move to the zero, restore the motor mode, move to MinPosition
  far      positive move with limit mode on; the MOTOR must stop at the far sensor (bit 3).
           For Y this requires X on its home sensor, where the door is.

``--driver-home`` instead runs init and then the real ``LinearAxis.home()`` in one call.

Throughout, a watchdog polls the motor and sends T (terminate) if the motor keeps moving
more than ``--overtravel`` ticks past the home-sensor edge, the encoder lags the commanded
position (stall), or an unexpected input bit changes.

  .venv\\Scripts\\python tools\\celigo_legacy\\home_stepwise.py --port COM4 --axis y --through init
"""

import argparse
import asyncio
import contextlib
import json
import logging
import sys
import time
from pathlib import Path
from typing import Any, Awaitable, Callable, Dict, List, Optional, TypeVar

from controller import connect, disconnect, make_celigo
from legacy_config import load_legacy_config

from pylabrobot.revvity.celigo import (
  Axis,
  Celigo,
  CeligoError,
  LinearAxis,
  LinearAxisConfig,
  StepperMotor,
)
from pylabrobot.revvity.celigo.protocol import complete_cleanup

STAGES = ("init", "units", "seek", "backoff", "index", "zero", "far")
HOME_TYPES = ("Normal", "Normal_Accurate", "NormalWithHardstopCheck")
SPECIAL_ENCODER_NO_INDEX = 1  # motion.py _EZ_SPECIAL_ENCODER_NO_INDEX
X_AXIS_ADDRESS = 1
HOME_SENSOR_BIT = 0x04  # opto 1, measured on X and Y
FAR_SENSOR_BIT = 0x08  # opto 2, not yet mapped
SEEK_DISTANCE = 25000  # the driver's fixed seek distance (motion.py LinearAxis.home)
UNITS_PROBE = 20
POLL_INTERVAL = 0.05
# At the draft speeds (<= 40 ticks/s) a poll sees a few ticks of motion; a bigger jump
# between polls is a counter reset (index home), not motion.
MAX_TICKS_PER_POLL = 100
# The index search drives into the home sensor and on to the next encoder index, which is
# at most one motor turn (200 ticks) past the sensor edge.
INDEX_OVERTRAVEL = 220
INDEX_ZERO_TOLERANCE = 5
CLEAR_MAX_TICKS = 500  # longest move used to drive off the home sensor before the seek
CLEAR_MARGIN_TICKS = 50
STALE_REPLY_WAIT = 0.1  # seconds for a cancelled command's reply to arrive before flushing

_T = TypeVar("_T")


class WatchdogTripped(CeligoError):
  pass


class Watchdog:
  """Poll one motor and terminate it if it misbehaves during a guarded operation."""

  def __init__(
    self,
    axis: Axis,
    stop_bits: int,
    overtravel_ticks: Optional[int],
    units_per_tick: Optional[float],
    stall_tolerance_ticks: int,
    max_travel_ticks: Optional[int] = None,
    overtravel_bit: int = HOME_SENSOR_BIT,
  ) -> None:
    self.axis = axis
    self.stop_bits = stop_bits
    self.overtravel_ticks = overtravel_ticks
    self.units_per_tick = units_per_tick
    self.stall_tolerance_ticks = stall_tolerance_ticks
    self.max_travel_ticks = max_travel_ticks
    self.overtravel_bit = overtravel_bit
    self.travelled_ticks = 0
    self.events: List[Dict[str, Any]] = []
    self.reason: Optional[str] = None

  async def _read(self, query: str) -> int:
    response = await self.axis.motor.send_command(query, execute=False)
    if not response.ok:
      raise CeligoError(f"watchdog {query} failed (code {response.error_code})")
    return int(response.response_text)

  async def run(self, started_at: float, stop: asyncio.Event) -> None:
    """Poll until ``stop`` is set. Each poll's queries always complete: a cancelled query
    leaves its reply in flight, and the next command would then read it as its own."""
    flags = await self._read("?4")
    units0 = last_units = await self._read("?0")
    ticks0 = last_ticks = await self._read("?8")
    sensor_edge_ticks = ticks0 if flags & self.overtravel_bit else None
    while True:
      with contextlib.suppress(asyncio.TimeoutError):
        await asyncio.wait_for(stop.wait(), POLL_INTERVAL)
      if stop.is_set():
        return
      ready = (await self.axis.motor.send_command("Q", execute=False)).ready
      current = await self._read("?4")
      units = await self._read("?0")
      ticks = await self._read("?8")
      jump = ticks - last_ticks
      if abs(jump) > MAX_TICKS_PER_POLL or (
        self.units_per_tick and abs(units - last_units) / self.units_per_tick > MAX_TICKS_PER_POLL
      ):
        # An index home zeroes the counters; re-reference instead of reading it as motion.
        self.events.append({"t": round(time.monotonic() - started_at, 3), "counter_reset": jump})
        units0, ticks0 = units, ticks
        if sensor_edge_ticks is not None:
          sensor_edge_ticks += jump
      else:
        self.travelled_ticks += abs(jump)
        if self.max_travel_ticks is not None and self.travelled_ticks > self.max_travel_ticks:
          await self.trip(f"travelled {self.travelled_ticks} ticks, limit {self.max_travel_ticks}")
      last_units, last_ticks = units, ticks
      if current != flags:
        self.events.append(
          {
            "t": round(time.monotonic() - started_at, 3),
            "from": flags,
            "to": current,
            "ticks": ticks,
            "units": units,
          }
        )
        if current & self.overtravel_bit and not flags & self.overtravel_bit:
          sensor_edge_ticks = ticks
        elif not current & self.overtravel_bit:
          sensor_edge_ticks = None
        if (current ^ flags) & self.stop_bits:
          await self.trip(f"input flags changed {flags} -> {current} at {ticks} ticks")
        flags = current
      if ready:
        # Measure stall lag per move, so reversal slack does not accumulate across moves.
        units0, ticks0 = units, ticks
        continue
      if (
        self.overtravel_ticks is not None
        and sensor_edge_ticks is not None
        and current & self.overtravel_bit
        and abs(ticks - sensor_edge_ticks) > self.overtravel_ticks
      ):
        await self.trip(
          f"motor kept moving {abs(ticks - sensor_edge_ticks)} ticks past the sensor edge"
        )
      if self.units_per_tick:
        lag = abs((units - units0) / self.units_per_tick - (ticks - ticks0))
        if lag > self.stall_tolerance_ticks:
          await self.trip(f"stall: encoder lags commanded position by {lag:.0f} ticks")

  async def trip(self, reason: str) -> None:
    self.reason = reason
    with contextlib.suppress(Exception):
      await complete_cleanup(self.axis.motor._terminate())
    raise WatchdogTripped(reason)


async def _cancel_and_flush(task: "asyncio.Future[Any]", axis: Axis) -> None:
  """Cancel a driver call and discard any reply still in flight from its last command."""
  task.cancel()
  with contextlib.suppress(BaseException):
    await task
  await asyncio.sleep(STALE_REPLY_WAIT)
  with contextlib.suppress(Exception):
    await axis._controller._board._purge_controller_buffers()


async def guarded(operation: Awaitable[_T], watchdog: Watchdog, started_at: float) -> _T:
  """Await a driver operation while ``watchdog`` runs; cancel it if the watchdog trips.

  The watchdog is stopped with an event, never cancelled, so it never abandons a query.
  """
  stop = asyncio.Event()
  task = asyncio.ensure_future(operation)
  monitor = asyncio.ensure_future(watchdog.run(started_at, stop))
  try:
    await asyncio.wait({task, monitor}, return_when=asyncio.FIRST_COMPLETED)
    if monitor.done():
      await _cancel_and_flush(task, watchdog.axis)
      monitor.result()  # re-raises WatchdogTripped or a watchdog query error
      raise CeligoError("watchdog stopped unexpectedly")
    stop.set()
    await monitor  # finishes its current poll, or raises if that poll tripped
    return task.result()
  except BaseException:
    stop.set()
    if not task.done():
      await _cancel_and_flush(task, watchdog.axis)
    with contextlib.suppress(BaseException):
      await monitor
    raise


async def read_state(axis: Axis) -> Dict[str, int]:
  values = {}
  for name, query in (("units", "?0"), ("encoder_ticks", "?8"), ("input_flags", "?4")):
    response = await axis.motor.send_command(query, execute=False)
    values[name] = int(response.response_text)
  return values


class Runner:
  def __init__(self, axis: LinearAxis, args: argparse.Namespace, report: Dict[str, Any]) -> None:
    self.axis = axis
    self.args = args
    self.report = report
    self.started_at = time.monotonic()
    # Stall detection needs the position-unit/encoder-tick ratio; the units stage measures
    # it, otherwise it comes from --units-per-tick.
    self.units_per_tick: Optional[float] = args.units_per_tick
    config = axis.config
    self.homing_mode = axis._motor_mode(enable_position_correction=False)
    self.max_velocity = axis._rate_to_encoder_tick_rate(config.max_velocity)
    self.homing_velocity = axis._rate_to_encoder_tick_rate(config.homing_velocity)
    self.index_velocity = axis._rate_to_encoder_tick_rate(config.index_velocity)

  def watchdog(
    self,
    stop_bits: int,
    overtravel: Optional[int] = None,
    overtravel_bit: int = HOME_SENSOR_BIT,
  ) -> Watchdog:
    return Watchdog(
      self.axis,
      stop_bits=stop_bits,
      overtravel_ticks=overtravel,
      units_per_tick=self.units_per_tick,
      stall_tolerance_ticks=self.args.stall_tolerance,
      overtravel_bit=overtravel_bit,
    )

  @property
  def uses_index(self) -> bool:
    return not self.axis.config.home_type.startswith("NormalWithHardstopCheck")

  async def stage(self, name: str, operation) -> None:
    result: Dict[str, Any] = {"before": await read_state(self.axis)}
    self.report["stages"][name] = result
    try:
      details = await operation()
      if details:
        result.update(details)
      result["outcome"] = "ok"
    except BaseException as exc:
      result["outcome"] = f"FAILED: {type(exc).__name__}: {exc}"
      raise
    finally:
      with contextlib.suppress(Exception):
        result["after"] = await read_state(self.axis)

  async def init(self) -> Dict[str, Any]:
    await self.axis._initialize()
    firmware = await self.axis.motor.request_motor_controller_firmware_version()
    return {
      "motor_firmware": firmware,
      "encoder_ratio_readback": await self.axis.request_encoder_ratio(),
      "accurate_index_supported": self.axis._supports_accurate_encoder_index,
    }

  async def units(self) -> Dict[str, Any]:
    await self.axis.motor._set_mode(self.homing_mode)
    before = await read_state(self.axis)
    await guarded(
      self.axis._move_homing_relative_ticks(True, UNITS_PROBE, self.homing_velocity),
      self.watchdog(FAR_SENSOR_BIT, self.args.overtravel),
      self.started_at,
    )
    middle = await read_state(self.axis)
    await guarded(
      self.axis._move_homing_relative_ticks(False, UNITS_PROBE, self.homing_velocity),
      self.watchdog(FAR_SENSOR_BIT, self.args.overtravel),
      self.started_at,
    )
    units_moved = middle["units"] - before["units"]
    ticks_moved = middle["encoder_ticks"] - before["encoder_ticks"]
    if ticks_moved >= UNITS_PROBE / 2:
      verdict = "encoder ticks (matches the driver)"
    elif units_moved == UNITS_PROBE and abs(ticks_moved) <= 1:
      verdict = "microsteps (driver distances would be 256x too short)"
    else:
      verdict = "unclear"
    if not verdict.startswith("encoder ticks"):
      raise CeligoError(f"position units are {verdict}; stopping before the seek")
    # One unit is one tick. A ratio taken from this short probe would carry a tick of slack
    # (e.g. 20/19), which grows into a false stall over a long move.
    self.units_per_tick = 1.0
    return {
      "units_moved": units_moved,
      "encoder_ticks_moved": ticks_moved,
      "units_per_tick": self.units_per_tick,
      "probe_ratio": units_moved / ticks_moved if ticks_moved else None,
      "verdict": verdict,
    }

  async def seek(self) -> Dict[str, Any]:
    await self.axis.motor._set_mode(self.homing_mode)
    if self.axis.config.s_curve_support:
      await self.axis.motor._set_parameter("aj", 0, "S-curve disable")
    await self.axis.motor._set_parameter("K", 0, "backlash disable")
    cleared = await self._clear_home_sensor()
    watchdog = self.watchdog(FAR_SENSOR_BIT, self.args.overtravel)
    await guarded(
      self.axis._move_homing_relative_ticks(False, SEEK_DISTANCE, self.homing_velocity),
      watchdog,
      self.started_at,
    )
    flags = await self.axis.request_limit_flags()
    if not flags & HOME_SENSOR_BIT:
      raise CeligoError("motor stopped without the home sensor active")
    return {
      "cleared_sensor_first": cleared,
      "sensor_events": watchdog.events,
      "stopped_by_motor_limit_mode": True,
    }

  async def _clear_home_sensor(self) -> Optional[Dict[str, Any]]:
    """Not part of the driver: drive off the home sensor so the seek meets its edge.

    ``home()`` backs off only ``HomingShortMove`` after the seek, so an axis that starts
    deeper inside the sensor than that cannot clear it.
    """
    if not await self.axis.request_limit_flags() & HOME_SENSOR_BIT:
      return None
    start = await read_state(self.axis)
    try:
      await guarded(
        self.axis._move_homing_relative_ticks(True, CLEAR_MAX_TICKS, self.homing_velocity),
        self.watchdog(FAR_SENSOR_BIT | HOME_SENSOR_BIT),
        self.started_at,
      )
    except WatchdogTripped:
      pass  # the expected stop: the home sensor changed (cleared)
    if await self.axis.request_limit_flags() & HOME_SENSOR_BIT:
      raise CeligoError(f"home sensor still active after {CLEAR_MAX_TICKS} ticks")
    cleared_at = await read_state(self.axis)
    await self.axis._move_homing_relative_ticks(True, CLEAR_MARGIN_TICKS, self.homing_velocity)
    # Includes the few ticks travelled between the edge and the watchdog stopping the motor.
    return {"sensor_depth_ticks": cleared_at["encoder_ticks"] - start["encoder_ticks"]}

  async def backoff(self) -> Dict[str, Any]:
    await asyncio.sleep(0.05)
    watchdog = self.watchdog(FAR_SENSOR_BIT)
    await guarded(
      self.axis._move_homing_relative_ticks(
        True, self.axis.config.homing_short_move, self.homing_velocity
      ),
      watchdog,
      self.started_at,
    )
    if await self.axis.request_limit_flags() & HOME_SENSOR_BIT:
      raise CeligoError("home sensor did not clear after backoff")
    return {"sensor_events": watchdog.events}

  async def index(self) -> Dict[str, Any]:
    await asyncio.sleep(0.05)
    config = self.axis.config
    if self.uses_index:
      accurate = (
        config.home_type == "Normal_Accurate" and self.axis._supports_accurate_encoder_index
      )
      special_mode = 6 if accurate else 2
      search = config.homing_short_move * 2
      # On this firmware Z zeroes the counters, drives back into the home sensor (limits are
      # off), then stops and zeroes again at the next encoder index past the sensor edge.
      watchdog = self.watchdog(FAR_SENSOR_BIT, INDEX_OVERTRAVEL)
    else:
      # As LinearAxis.home() for NormalWithHardstopCheck: no index; Z must stop at the edge.
      special_mode = SPECIAL_ENCODER_NO_INDEX
      search = SEEK_DISTANCE
      watchdog = self.watchdog(FAR_SENSOR_BIT, self.args.overtravel)
    await guarded(
      self.axis._home_to_encoder_index(
        search,
        self.index_velocity,
        special_mode,
        timeout=max(30.0, search / self.index_velocity + 2.0),
        restore_motor_mode=self.homing_mode,
      ),
      watchdog,
      self.started_at,
    )
    state = await read_state(self.axis)
    details = {"special_mode": special_mode, "search_ticks": search, "events": watchdog.events}
    if not self.uses_index and not await self.axis.request_limit_flags() & HOME_SENSOR_BIT:
      raise CeligoError(f"Z ended off the home sensor (events: {watchdog.events})")
    if abs(state["units"]) > INDEX_ZERO_TOLERANCE:
      raise CeligoError(
        f"no encoder index found: Z ended at {state['units']} units instead of zeroing at an "
        f"index (events: {watchdog.events})"
      )
    return details

  async def zero(self) -> Dict[str, Any]:
    # The zero lies at or inside the home sensor, so only the far sensor stops these moves.
    watchdog = self.watchdog(FAR_SENSOR_BIT, INDEX_OVERTRAVEL)
    at_index = await guarded(
      Axis.move_to_ticks(self.axis, 0, velocity_ticks_per_second=self.max_velocity),
      watchdog,
      self.started_at,
    )
    await self.axis._restore_homing_configuration()
    settled = await guarded(
      Axis.move_to_ticks(self.axis, self.axis.mm_to_encoder_ticks(self.axis.config.min_position)),
      self.watchdog(FAR_SENSOR_BIT, INDEX_OVERTRAVEL),
      self.started_at,
    )
    self.axis._has_position_reference = True
    return {
      "settled_at_index_ticks": at_index,
      "settled_at_min_position_ticks": settled,
      "position_mm": await self.axis.request_position(),
    }

  async def far(self) -> Dict[str, Any]:
    """Not part of home(): confirm limit mode stops a positive move at the far sensor."""
    if not self.axis.config.positive_limit:
      raise CeligoError(f"axis {self.axis.name} has no far sensor configured")
    if self.axis.name == "y":
      x_flags = await _motor_input_flags(self.axis, X_AXIS_ADDRESS)
      if not x_flags & HOME_SENSOR_BIT:
        raise CeligoError("Y exits through the door only with X on its home sensor")
    await self.axis.motor._set_mode(self.axis._motor_mode())
    watchdog = self.watchdog(HOME_SENSOR_BIT, self.args.overtravel, overtravel_bit=FAR_SENSOR_BIT)
    await guarded(
      self.axis._move_homing_relative_ticks(True, SEEK_DISTANCE, self.max_velocity),
      watchdog,
      self.started_at,
    )
    if not await self.axis.request_limit_flags() & FAR_SENSOR_BIT:
      raise CeligoError("motor stopped without the far sensor active")
    return {
      "sensor_events": watchdog.events,
      "position_mm": await self.axis.request_position(),
      "stopped_by_motor_limit_mode": True,
    }

  async def driver_home(self) -> Dict[str, Any]:
    # An index home travels past the sensor edge; a sensor-edge home must stop at it.
    overtravel = INDEX_OVERTRAVEL if self.uses_index else self.args.overtravel
    # An axis starting on its far sensor (e.g. Y out at the door) leaves it while homing.
    on_far_sensor = bool(await self.axis.request_limit_flags() & FAR_SENSOR_BIT)
    watchdog = self.watchdog(0 if on_far_sensor else FAR_SENSOR_BIT, overtravel)
    settled = await guarded(self.axis.home(), watchdog, self.started_at)
    return {
      "settled_ticks": settled,
      "position_mm": await self.axis.request_position(),
      "sensor_events": watchdog.events,
    }


async def _motor_input_flags(axis: Axis, address: int) -> int:
  """Raw input flags of another motor on the same controller (a query, no motion)."""
  response = await StepperMotor(axis._controller, address).send_command("?4", execute=False)
  if not response.ok:
    raise CeligoError(f"motor {address} ?4 failed (code {response.error_code})")
  return int(response.response_text)


def _axis_config(celigo: Celigo, axis: str) -> LinearAxisConfig:
  configs: Dict[str, Optional[LinearAxisConfig]] = {
    "x": celigo.config.hardware.x_axis,
    "y": celigo.config.hardware.y_axis,
    "z": celigo.config.hardware.z_axis,
  }
  config = configs[axis]
  if config is None:
    raise CeligoError(f"axis {axis} is not configured")
  return config


async def run(args: argparse.Namespace, report: Dict[str, Any]) -> None:
  celigo = make_celigo(args.port, config=load_legacy_config(), reply_timeout=args.reply_timeout)
  try:
    await connect(celigo)
  except CeligoError as exc:
    report["handshake"] = f"FAILED: {exc}"
    return
  report["handshake"] = "ok"
  report["stages"] = {}
  try:
    config = _axis_config(celigo, args.axis)
    if args.home_type is not None:
      config.home_type = args.home_type
    report["home_type"] = config.home_type
    detected = {motor.motor_index for motor in await celigo.request_detected_motor_addresses()}
    if config.axis_index not in detected:
      raise CeligoError(f"motor {config.axis_index} was not detected")
    axis = LinearAxis(celigo.motor_controller, args.axis, config)
    runner = Runner(axis, args, report)
    stages: Dict[str, Callable[[], Awaitable[Dict[str, Any]]]] = {
      "units": runner.units,
      "seek": runner.seek,
      "backoff": runner.backoff,
      "index": runner.index,
      "zero": runner.zero,
      "far": runner.far,
    }
    await runner.stage("init", runner.init)
    if args.driver_home:
      await runner.stage("driver_home", runner.driver_home)
    else:
      for name in STAGES[1 : STAGES.index(args.through) + 1]:
        await runner.stage(name, stages[name])
  except (CeligoError, TimeoutError, ValueError) as exc:
    report["error"] = f"{type(exc).__name__}: {exc}"
  finally:
    await disconnect(celigo)


def main(argv: Optional[List[str]] = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument("--port", required=True, help="controller COM port, e.g. COM4")
  parser.add_argument("--axis", choices=("x", "y", "z"), default="y")
  parser.add_argument("--home-type", choices=HOME_TYPES, help="override the configured HomeType")
  group = parser.add_mutually_exclusive_group(required=True)
  group.add_argument("--through", choices=STAGES, help="run stages up to and including this one")
  group.add_argument("--driver-home", action="store_true", help="init, then LinearAxis.home()")
  parser.add_argument(
    "--units-per-tick",
    type=float,
    help="measured position units per encoder tick (required with --driver-home)",
  )
  parser.add_argument("--overtravel", type=int, default=30, help="ticks allowed past the sensor")
  parser.add_argument(
    "--stall-tolerance", type=int, default=30, help="allowed encoder lag per move (ticks)"
  )
  parser.add_argument("--reply-timeout", type=float, default=2.0)
  parser.add_argument(
    "--out",
    type=Path,
    default=Path(__file__).resolve().parent / "runs",
    help="directory for the report and byte log",
  )
  args = parser.parse_args(argv)
  if args.driver_home and not args.units_per_tick:
    parser.error("--driver-home requires --units-per-tick from a units-stage run")

  args.out.mkdir(parents=True, exist_ok=True)
  stamp = time.strftime("%Y%m%d-%H%M%S")
  label = "driver_home" if args.driver_home else args.through
  name = f"celigo_home_{args.axis}_{label}_{stamp}"
  logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s %(name)s %(message)s",
    handlers=[logging.FileHandler(args.out / f"{name}.log", encoding="utf-8")],
  )
  report: Dict[str, Any] = {
    "port": args.port,
    "axis": args.axis,
    "through": label,
    "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
  }
  try:
    asyncio.run(run(args, report))
  except KeyboardInterrupt:
    report["interrupted"] = True
  report_path = args.out / f"{name}.json"
  report_path.write_text(json.dumps(report, indent=2, default=str), encoding="utf-8")
  print(json.dumps(report, indent=2, default=str))
  print(f"\nreport: {report_path}\nbyte log: {args.out / f'{name}.log'}")
  return 0 if report.get("handshake") == "ok" and "error" not in report else 1


if __name__ == "__main__":
  sys.exit(main())
