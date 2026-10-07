"""Move ONE motor a relative distance and leave it there (no return move).

Use only on a motor whose direction and sensors have been mapped with ``wiggle_motor.py``.
The move stops with ``T`` if:

* any input flag outside ``--ignore-bits`` changes (e.g. an unmapped end-of-travel sensor);
* the encoder lags the commanded steps by more than ``--stall-tolerance`` ticks, given the
  measured ``--microsteps-per-tick`` (e.g. the stage reached a hard stop);
* an error, timeout, or Ctrl+C occurs.

The motor holds its new position afterwards. To undo, jog the same distance the other way.

  .venv\\Scripts\\python tools\\celigo_legacy\\jog_motor.py --port COM4 --address 1 \\
    --turns 5 --ignore-bits 4 --microsteps-per-tick 256
"""

import argparse
import asyncio
import contextlib
import json
import logging
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from controller import connect, disconnect, make_celigo
from wiggle_motor import (
  MAX_CURRENT_PERCENT,
  _bounded_int,
  _move,
  read_state,
  require_microstep_units,
)

from pylabrobot.revvity.celigo import CeligoError, StepperMotor
from pylabrobot.revvity.celigo.protocol import complete_cleanup

MICROSTEPS_PER_TURN = 51200  # 200 steps/rev x 256 microsteps/step, confirmed by encoder runs
# Raised once each axis's travel was mapped: X spans ~51 turns between its sensors.
MAX_TURNS = 60.0
MAX_VELOCITY_MICROSTEPS_PER_SECOND = 51200  # one motor turn per second


async def jog(
  motor: StepperMotor,
  distance: int,
  current_percent: int,
  velocity: int,
  ignored_flag_bits: int,
  microsteps_per_tick: float,
  stall_tolerance_ticks: int,
  report: Dict[str, Any],
) -> None:
  started_at = time.monotonic()
  flag_changes: List[Dict[str, Any]] = []
  report["flag_changes"] = flag_changes
  await require_microstep_units(motor)
  status = await motor.send_command("Q", execute=False)
  if not status.ok or not status.ready:
    raise CeligoError(f"motor {motor.axis_index} is not ready (error {status.error_code})")
  start = await read_state(motor)
  report["start"] = start
  try:
    report["stopped_on_flag_change"] = await _move(
      motor,
      distance,
      current_percent,
      velocity,
      started_at,
      flag_changes,
      stop_on_flag_change=True,
      ignored_flag_bits=ignored_flag_bits,
      stall_check=(microsteps_per_tick, stall_tolerance_ticks),
    )
  except BaseException:
    with contextlib.suppress(Exception):
      await complete_cleanup(motor._terminate())
    raise
  finally:
    with contextlib.suppress(Exception):
      report["end"] = await read_state(motor)
  end = report["end"]
  taken = end["motor_microsteps"] - start["motor_microsteps"]
  report["summary"] = {
    "microsteps_commanded": distance,
    "microsteps_taken": taken,
    "turns_taken": round(taken / MICROSTEPS_PER_TURN, 3),
    "encoder_ticks": end["encoder_ticks"] - start["encoder_ticks"],
  }


async def run(args: argparse.Namespace, report: Dict[str, Any]) -> None:
  celigo = make_celigo(args.port, reply_timeout=args.reply_timeout)
  try:
    await connect(celigo)
  except CeligoError as exc:
    report["handshake"] = f"FAILED: {exc}"
    return
  try:
    report["handshake"] = "ok"
    detected = {motor.motor_index for motor in await celigo.request_detected_motor_addresses()}
    if args.address not in detected:
      raise CeligoError(f"motor {args.address} was not detected; detected: {sorted(detected)}")
    distance = round(args.turns * MICROSTEPS_PER_TURN)
    if args.direction == "negative":
      distance = -distance
    await jog(
      StepperMotor(celigo.motor_controller, args.address),
      distance,
      args.current,
      args.velocity,
      args.ignore_bits,
      args.microsteps_per_tick,
      args.stall_tolerance,
      report,
    )
  except (CeligoError, TimeoutError) as exc:
    report["error"] = f"{type(exc).__name__}: {exc}"
  finally:
    await disconnect(celigo)


def _turns(text: str) -> float:
  value = float(text)
  if not 0 < value <= MAX_TURNS:
    raise argparse.ArgumentTypeError(f"turns must be in (0, {MAX_TURNS:g}]")
  return value


def main(argv: Optional[List[str]] = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument("--port", required=True, help="controller COM port, e.g. COM4")
  parser.add_argument("--address", required=True, type=int, help="the one motor to move")
  parser.add_argument("--direction", choices=("positive", "negative"), default="positive")
  parser.add_argument("--turns", required=True, type=_turns, help=f"motor turns, max {MAX_TURNS:g}")
  parser.add_argument(
    "--ignore-bits",
    type=_bounded_int("ignore-bits", 0, 15),
    default=0,
    help="input-flag bits whose changes are logged but do not stop (e.g. 4 = bit 2)",
  )
  parser.add_argument(
    "--microsteps-per-tick",
    required=True,
    type=float,
    help="measured microsteps per encoder tick for this motor (stall detection)",
  )
  parser.add_argument(
    "--stall-tolerance", type=int, default=20, help="allowed encoder lag in ticks (default 20)"
  )
  parser.add_argument(
    "--current",
    type=_bounded_int("current", 1, MAX_CURRENT_PERCENT),
    default=30,
    help=f"move current, percent of rated (default 30, max {MAX_CURRENT_PERCENT})",
  )
  parser.add_argument(
    "--velocity",
    type=_bounded_int("velocity", 1, MAX_VELOCITY_MICROSTEPS_PER_SECOND),
    default=10240,
    help="microsteps per second (default 10240 = 0.2 turn/s, max 51200 = 1 turn/s)",
  )
  parser.add_argument("--reply-timeout", type=float, default=2.0)
  parser.add_argument(
    "--out",
    type=Path,
    default=Path(__file__).resolve().parent / "runs",
    help="directory for the report and byte log",
  )
  args = parser.parse_args(argv)
  if args.microsteps_per_tick <= 0:
    parser.error("--microsteps-per-tick must be positive")

  args.out.mkdir(parents=True, exist_ok=True)
  stamp = time.strftime("%Y%m%d-%H%M%S")
  name = f"celigo_jog_m{args.address}_{stamp}"
  logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s %(name)s %(message)s",
    handlers=[logging.FileHandler(args.out / f"{name}.log", encoding="utf-8")],
  )
  report: Dict[str, Any] = {
    "port": args.port,
    "address": args.address,
    "direction": args.direction,
    "turns": args.turns,
    "ignore_bits": args.ignore_bits,
    "microsteps_per_tick": args.microsteps_per_tick,
    "stall_tolerance": args.stall_tolerance,
    "current_percent": args.current,
    "velocity": args.velocity,
    "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
  }
  try:
    asyncio.run(run(args, report))
  except KeyboardInterrupt:
    report["interrupted"] = True
  report_path = args.out / f"{name}.json"
  report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
  print(json.dumps(report, indent=2))
  print(f"\nreport: {report_path}\nbyte log: {args.out / f'{name}.log'}")
  return 0 if report.get("handshake") == "ok" and "error" not in report else 1


if __name__ == "__main__":
  sys.exit(main())
