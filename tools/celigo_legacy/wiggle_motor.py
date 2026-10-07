"""Step 2b: move ONE motor a small distance out and back, to identify its mechanism.

This is the first tool that drives a motor. For the single ``--address`` given it:

1. reads the encoder (``?8``), motor step counter (``?0``), and input flags (``?4``);
2. sends ``m<current>V<velocity>P<distance>R`` (or ``D`` for ``--direction negative``);
3. polls until the motor is ready, stopping it with ``T`` if any input flag changes
   (``--continue-past-flags`` only logs the change; use it once a motor's sensors are known);
4. waits ``--pause`` seconds, then moves back by the steps actually taken (from ``?0``);
5. reads the encoder, step counter, and flags again.

Units are motor microsteps. Limit switches are not enabled on a freshly powered motor,
so the move is kept short and slow at a capped current: at a hard stop a stepper stalls
rather than pushing hard. Any error or Ctrl+C sends ``T`` (terminate) to the motor.

The move current set here stays in the motor's RAM until power-off; the hold current
is not changed.

  .venv\\Scripts\\python tools\\celigo_legacy\\wiggle_motor.py --port COM4 --address 2
"""

import argparse
import asyncio
import contextlib
import json
import logging
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from controller import connect, disconnect, make_celigo

from pylabrobot.revvity.celigo import CeligoError, StepperMotor
from pylabrobot.revvity.celigo.protocol import complete_cleanup

MAX_DISTANCE_MICROSTEPS = 51200  # one turn at 200 steps/rev and 256 microsteps/step
MAX_CURRENT_PERCENT = 50
MAX_VELOCITY_MICROSTEPS_PER_SECOND = 12800
POLL_INTERVAL = 0.05


async def _read_int(motor: StepperMotor, query: str) -> int:
  response = await motor.send_command(query, execute=False)
  if not response.ok:
    raise CeligoError(f"motor {motor.axis_index} {query} failed (code {response.error_code})")
  return int(response.response_text)


async def require_microstep_units(motor: StepperMotor) -> None:
  """Refuse to move a motor the driver has configured.

  The scripts here command microsteps. ``Axis._initialize()`` (run by ``home_stepwise.py``)
  sets the encoder ratio, after which the motor takes positions in encoder ticks, so the
  same distance would be 256 times larger on the stage motors. Power-up ratio is 1000.
  """
  response = await motor.send_command("?aE", execute=False)
  if not response.ok or response.response_text.strip() != "1000":
    raise CeligoError(
      f"motor {motor.axis_index} encoder ratio is {response.response_text!r}, not the power-up "
      "1000: the driver has configured it and distances are no longer microsteps. "
      "Power-cycle the instrument before using this script on it."
    )


async def read_state(motor: StepperMotor) -> Dict[str, int]:
  return {
    "encoder_ticks": await _read_int(motor, "?8"),
    "motor_microsteps": await _read_int(motor, "?0"),
    "input_flags": await _read_int(motor, "?4"),
  }


async def _move(
  motor: StepperMotor,
  distance: int,
  current_percent: int,
  velocity: int,
  started_at: float,
  flag_changes: List[Dict[str, Any]],
  stop_on_flag_change: bool,
  ignored_flag_bits: int = 0,
  stall_check: Optional[Tuple[float, int]] = None,
) -> bool:
  """Move ``distance`` microsteps (signed) and wait. Returns True if stopped on a flag change.

  Changes confined to ``ignored_flag_bits`` are logged but never stop the move.
  ``stall_check=(microsteps_per_tick, tolerance_ticks)`` stops the motor and raises if the
  encoder lags the commanded steps by more than the tolerance (e.g. at a hard stop).
  """
  direction = "P" if distance > 0 else "D"
  response = await motor.send_command(f"m{current_percent}V{velocity}{direction}{abs(distance)}")
  if not response.ok:
    raise CeligoError(f"motor {motor.axis_index} rejected the move (code {response.error_code})")
  timeout = abs(distance) / velocity + 5.0
  deadline = time.monotonic() + timeout
  flags = await _read_int(motor, "?4")
  if stall_check is not None:
    start_microsteps = await _read_int(motor, "?0")
    start_ticks = await _read_int(motor, "?8")
  while True:
    status = await motor.send_command("Q", execute=False)
    if not status.ok:
      raise CeligoError(f"motor {motor.axis_index} reported error {status.error_code}")
    current_flags = await _read_int(motor, "?4")
    if current_flags != flags:
      flag_changes.append(
        {
          "t": round(time.monotonic() - started_at, 3),
          "from": flags,
          "to": current_flags,
          "motor_microsteps": await _read_int(motor, "?0"),
          "encoder_ticks": await _read_int(motor, "?8"),
        }
      )
      stopping_bits = (current_flags ^ flags) & ~ignored_flag_bits
      flags = current_flags
      if stop_on_flag_change and stopping_bits and not status.ready:
        await motor._terminate()
        return True
    if stall_check is not None:
      microsteps_per_tick, tolerance_ticks = stall_check
      expected_ticks = (await _read_int(motor, "?0") - start_microsteps) / microsteps_per_tick
      lag_ticks = abs(expected_ticks - (await _read_int(motor, "?8") - start_ticks))
      if lag_ticks > tolerance_ticks:
        await motor._terminate()
        raise CeligoError(
          f"motor {motor.axis_index} stalled: encoder lags commanded steps by "
          f"{lag_ticks:.0f} ticks (tolerance {tolerance_ticks})"
        )
    if status.ready:
      return False
    if time.monotonic() >= deadline:
      raise TimeoutError(f"motor {motor.axis_index} did not finish within {timeout:.1f} s")
    await asyncio.sleep(POLL_INTERVAL)


async def wiggle(
  motor: StepperMotor,
  distance: int,
  current_percent: int,
  velocity: int,
  pause: float,
  report: Dict[str, Any],
  stop_on_flag_change: bool = True,
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
    report["outward_stopped_on_flag_change"] = await _move(
      motor, distance, current_percent, velocity, started_at, flag_changes, stop_on_flag_change
    )
    outward = await read_state(motor)
    report["outward"] = outward
    await asyncio.sleep(pause)
    # Return by the steps actually taken, which differs from ``distance`` after an early stop.
    taken = outward["motor_microsteps"] - start["motor_microsteps"]
    if taken:
      await _move(motor, -taken, current_percent, velocity, started_at, flag_changes, False)
  except BaseException:
    with contextlib.suppress(Exception):
      await complete_cleanup(motor._terminate())
    raise
  end = await read_state(motor)
  report["end"] = end
  outward_encoder = outward["encoder_ticks"] - start["encoder_ticks"]
  report["summary"] = {
    "microsteps_commanded": distance,
    "microsteps_taken": taken,
    "encoder_ticks_outward": outward_encoder,
    "encoder_ticks_per_microstep": outward_encoder / taken if taken else None,
    "returned_microsteps": end["motor_microsteps"] - start["motor_microsteps"] == 0,
    "encoder_offset_after_return": end["encoder_ticks"] - start["encoder_ticks"],
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
    motor = StepperMotor(celigo.motor_controller, args.address)
    signed_distance = args.distance if args.direction == "positive" else -args.distance
    await wiggle(
      motor,
      signed_distance,
      args.current,
      args.velocity,
      args.pause,
      report,
      stop_on_flag_change=not args.continue_past_flags,
    )
  except (CeligoError, TimeoutError) as exc:
    report["error"] = f"{type(exc).__name__}: {exc}"
  finally:
    await disconnect(celigo)


def _bounded_int(name: str, low: int, high: int):
  def parse(text: str) -> int:
    value = int(text)
    if not low <= value <= high:
      raise argparse.ArgumentTypeError(f"{name} must be in {low}..{high}")
    return value

  return parse


def main(argv: Optional[List[str]] = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument("--port", required=True, help="controller COM port, e.g. COM4")
  parser.add_argument("--address", required=True, type=int, help="the one motor to move")
  parser.add_argument("--direction", choices=("positive", "negative"), default="positive")
  parser.add_argument(
    "--distance",
    type=_bounded_int("distance", 1, MAX_DISTANCE_MICROSTEPS),
    default=2560,
    help=f"microsteps each way (default 2560, max {MAX_DISTANCE_MICROSTEPS})",
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
    default=2560,
    help=f"microsteps per second (default 2560, max {MAX_VELOCITY_MICROSTEPS_PER_SECOND})",
  )
  parser.add_argument(
    "--continue-past-flags",
    action="store_true",
    help="log input-flag changes without stopping (only for motors with mapped sensors)",
  )
  parser.add_argument("--pause", type=float, default=1.0, help="seconds between out and back")
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
  name = f"celigo_wiggle_m{args.address}_{stamp}"
  logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s %(name)s %(message)s",
    handlers=[logging.FileHandler(args.out / f"{name}.log", encoding="utf-8")],
  )
  report: Dict[str, Any] = {
    "port": args.port,
    "address": args.address,
    "direction": args.direction,
    "distance": args.distance,
    "current_percent": args.current,
    "velocity": args.velocity,
    "continue_past_flags": args.continue_past_flags,
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
