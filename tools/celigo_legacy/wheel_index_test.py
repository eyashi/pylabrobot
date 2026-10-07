"""Check whether the legacy Celigo's filter-wheel encoder has an index pulse.

Runs the driver's own ``FilterWheel`` initialization and its index search
(``Axis._home_to_encoder_index``, as used by ``FilterWheel.home()``) under the stepwise
homing watchdog. On this firmware ``Z`` first seeks the home opto (the position-one flag,
once per wheel turn = 8000 ticks), then the next encoder index (at most one motor turn =
2000 ticks), and does not honor its distance argument; so the watchdog stops the wheel
after ``--max-travel`` ticks and that outcome means "no index".

With ``--map``, once an index is found it also turns the wheel one revolution recording
the position-one flag, sets ``HomeOffset`` to the flag center (modulo one position), runs
the driver's own ``FilterWheel.home()`` with it, and visits logical positions 1-4.

  .venv\\Scripts\\python tools\\celigo_legacy\\wheel_index_test.py --port COM4 --map
"""

import argparse
import asyncio
import json
import logging
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from controller import connect, disconnect, make_celigo
from home_stepwise import Watchdog, WatchdogTripped, guarded, read_state

from pylabrobot.revvity.celigo import (
  Axis,
  CeligoError,
  FilterMapEntry,
  FilterWheel,
  FilterWheelConfig,
)

WHEEL_ADDRESS = 4
TICKS_PER_REVOLUTION = 8000
FILTERS = 4
MICROSTEPS_PER_TICK = 25.6  # 51200 microsteps per motor turn / 2000 ticks
INDEX_VELOCITY = 200  # ticks/s: optical axes take encoder rates directly
TICKS_PER_POSITION = TICKS_PER_REVOLUTION // FILTERS
MAP_VELOCITY = 400
FLAG_BIT = 0x04  # raw bit 2 clears while the position-one flag is in the sensor


def wheel_config() -> FilterWheelConfig:
  return FilterWheelConfig(
    motion_name="Dichroic (legacy draft)",
    config_version=1001,
    motor_type=0,
    comm_index=1,
    controller_index=0,
    axis_index=WHEEL_ADDRESS,
    enabled=True,
    max_velocity=400.0,
    max_acceleration=400.0,
    max_deceleration=400.0,
    max_s_acceleration=0,
    moderate_acceleration=400.0,
    minimum_acceleration=400.0,
    moderate_s_acceleration=0,
    minimum_s_acceleration=0,
    s_curve_support=False,
    home_type="Normal_Accurate",
    homing_velocity=400.0,
    index_velocity=float(INDEX_VELOCITY),
    homing_short_move=200,
    home_offset=0.0,
    positive_limit=False,
    negative_limit=False,
    limit_polarity=1,  # the position-one flag clears bit 2
    invert_axis_direction=False,
    default_positive_direction=True,
    moving_current_percentage=30,
    holding_current_percentage=10,
    loading_current_percentage=30,
    moving_overload_limit=500,
    mode_enable_limits=False,
    mode_enable_step_and_direction=False,
    mode_enable_position_correction=False,
    mode_enable_motor_slave_to_encoder=False,
    coarse_position_error_window=100,
    fine_position_error_window=10,
    gain=1,
    encoder_to_motor_tick_ratio=MICROSTEPS_PER_TICK,
    backlash_compensation=0,
    motor_response_time=1,
    encoder_ticks_per_revolution=TICKS_PER_REVOLUTION,
    number_of_filters=FILTERS,
    filter_map=[FilterMapEntry(logical_number=n, physical_number=n) for n in range(1, 5)],
  )


def _wheel_watchdog(wheel: FilterWheel, max_travel: int) -> Watchdog:
  return Watchdog(
    wheel,
    stop_bits=0,
    overtravel_ticks=None,
    units_per_tick=1.0,
    stall_tolerance_ticks=40,
    max_travel_ticks=max_travel,
  )


async def map_flag(wheel: FilterWheel) -> Dict[str, Any]:
  """Turn one revolution and derive HomeOffset from the position-one flag's center."""
  start = await read_state(wheel)
  watchdog = _wheel_watchdog(wheel, TICKS_PER_REVOLUTION + 600)
  await guarded(
    Axis.move_to_ticks(
      wheel, start["units"] + TICKS_PER_REVOLUTION, velocity_ticks_per_second=MAP_VELOCITY
    ),
    watchdog,
    time.monotonic(),
  )
  flag_changes = [event for event in watchdog.events if "from" in event]
  entered = [e for e in flag_changes if e["from"] & FLAG_BIT and not e["to"] & FLAG_BIT]
  left = [e for e in flag_changes if not e["from"] & FLAG_BIT and e["to"] & FLAG_BIT]
  if len(entered) != 1 or len(left) != 1:
    raise CeligoError(f"expected one flag passage in one revolution, saw {flag_changes}")
  center = (entered[0]["units"] + left[0]["units"]) / 2
  return {
    "flag_entered_units": entered[0]["units"],
    "flag_left_units": left[0]["units"],
    "flag_width_ticks": left[0]["units"] - entered[0]["units"],
    "home_offset": round(center) % TICKS_PER_POSITION,
  }


async def driver_home(wheel: FilterWheel, home_offset: int) -> Dict[str, Any]:
  """FilterWheel.home() with the mapped offset, then visit every logical position."""
  wheel.config.home_offset = float(home_offset)
  watchdog = _wheel_watchdog(wheel, 3 * TICKS_PER_REVOLUTION)
  home_ticks = await guarded(wheel.home(), watchdog, time.monotonic())
  positions = {}
  for logical in range(1, FILTERS + 1):
    settled = await guarded(wheel.move_to(logical), _wheel_watchdog(wheel, 9000), time.monotonic())
    flags = await wheel.request_limit_flags()  # polarity-corrected: bit 2 set = at the flag
    positions[logical] = {"settled_ticks": settled, "at_flag": bool(flags & FLAG_BIT)}
  return {"home_ticks": home_ticks, "positions": positions}


async def run(args: argparse.Namespace, report: Dict[str, Any]) -> None:
  celigo = make_celigo(args.port, reply_timeout=args.reply_timeout)
  try:
    await connect(celigo)
  except CeligoError as exc:
    report["handshake"] = f"FAILED: {exc}"
    return
  try:
    report["handshake"] = "ok"
    wheel = FilterWheel(celigo.motor_controller, "dichroic_filter", wheel_config())
    await wheel._initialize()
    report["after_init"] = await read_state(wheel)
    report["encoder_ratio_readback"] = await wheel.request_encoder_ratio()
    special_mode = 6 if wheel._supports_accurate_encoder_index else 2
    report["special_mode"] = special_mode
    watchdog = Watchdog(
      wheel,
      stop_bits=0,  # the flag passing is expected
      overtravel_ticks=None,
      units_per_tick=1.0,
      stall_tolerance_ticks=40,
      max_travel_ticks=args.max_travel,
    )
    started = time.monotonic()
    try:
      await guarded(
        wheel._home_to_encoder_index(
          round(TICKS_PER_REVOLUTION / FILTERS * 1.2),
          INDEX_VELOCITY,
          special_mode,
          timeout=args.max_travel / INDEX_VELOCITY + 10.0,
        ),
        watchdog,
        started,
      )
      stopped_by = "motor"
    except WatchdogTripped as exc:
      stopped_by = f"watchdog: {exc}"
    state = await read_state(wheel)
    report["search"] = {
      "stopped_by": stopped_by,
      "seconds": round(time.monotonic() - started, 2),
      "travelled_ticks": watchdog.travelled_ticks,
      "end_state": state,
      "events": watchdog.events,
    }
    # The motor stops by itself within one motor turn only at an index; this firmware sets
    # the counters to fixed non-zero values there, so the end position is not checked.
    report["has_index"] = stopped_by == "motor" and watchdog.travelled_ticks <= TICKS_PER_POSITION
    if args.map and report["has_index"]:
      report["map"] = await map_flag(wheel)
      report["driver_home"] = await driver_home(wheel, report["map"]["home_offset"])
  except (CeligoError, TimeoutError) as exc:
    report["error"] = f"{type(exc).__name__}: {exc}"
  finally:
    await disconnect(celigo)


def main(argv: Optional[List[str]] = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument("--port", required=True, help="controller COM port, e.g. COM4")
  parser.add_argument(
    "--max-travel", type=int, default=10500, help="ticks before the watchdog stops the wheel"
  )
  parser.add_argument(
    "--map", action="store_true", help="also map the flag and run FilterWheel.home()"
  )
  parser.add_argument("--reply-timeout", type=float, default=2.0)
  parser.add_argument(
    "--out",
    type=Path,
    default=Path(__file__).resolve().parent / "runs",
    help="directory for the report and byte log",
  )
  args = parser.parse_args(argv)

  args.out.mkdir(parents=True, exist_ok=True)
  stamp = time.strftime("%Y%m%d-%H%M%S")
  name = f"celigo_wheel_index_{stamp}"
  logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s %(name)s %(message)s",
    handlers=[logging.FileHandler(args.out / f"{name}.log", encoding="utf-8")],
  )
  report: Dict[str, Any] = {"port": args.port, "started_at": time.strftime("%Y-%m-%dT%H:%M:%S")}
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
