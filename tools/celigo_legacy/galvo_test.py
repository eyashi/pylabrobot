"""Exercise the legacy Celigo's galvo mirrors through the driver's Galvo component.

Stages (``--through``):

  calibrate  set each galvo's settling window, then run the driver's calibration sweep
             twice per axis, exactly as ``Galvo._initialize()`` does at setup; record the
             result, the galvo status, and the calibration data before and after
  move       after calibrating: step each mirror +step, -step, then back to 0 V, waiting
             for the controller to report it settled each time

The draft galvo configuration limits targets to +/-``--max-volts`` (default 1 V) with no
inversion, so logical and hardware voltages are equal. The laser is never touched.

  .venv\\Scripts\\python tools\\celigo_legacy\\galvo_test.py --port COM4 --through calibrate
"""

import argparse
import asyncio
import json
import logging
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from controller import connect, disconnect, make_celigo

from pylabrobot.revvity.celigo import Celigo, CeligoError, GalvoConfig

AXES: Tuple[str, ...] = ("x", "y")
# Settling windows from the trimmed vendor sample (tests/config_tests.py).
POSITION_ERROR_WINDOW = 20
VELOCITY_ERROR_WINDOW = 20
CALIBRATION_TIMEOUT = 0.9  # seconds, as in Galvo._initialize()
MAX_STEP_VOLTS = 2.0


def draft_galvo_config(max_volts: float) -> GalvoConfig:
  return GalvoConfig(
    config_version=1000,
    controller_index=0,
    position_error_window=POSITION_ERROR_WINDOW,
    velocity_error_window=VELOCITY_ERROR_WINDOW,
    big_move_delay=0.0,
    min_voltage=-max_volts,
    max_voltage=max_volts,
    invert_voltage=False,
    enabled=True,
  )


def _summarize_errors(pairs: List[Tuple[int, int]]) -> Dict[str, Any]:
  if not pairs:
    return {"count": 0}
  firsts = [pair[0] for pair in pairs]
  seconds = [pair[1] for pair in pairs]
  return {
    "count": len(pairs),
    "first_range": [min(firsts), max(firsts)],
    "second_range": [min(seconds), max(seconds)],
    "head": pairs[:5],
  }


async def _status(celigo: Celigo) -> Dict[str, Any]:
  return asdict(await celigo.galvo.request_controller_status())


async def calibrate(celigo: Celigo) -> Dict[str, Any]:
  result: Dict[str, Any] = {"status_before": await _status(celigo)}
  for axis in AXES:
    result[f"{axis}_errors_before"] = _summarize_errors(
      await celigo.galvo.request_calibration_errors(axis)  # type: ignore[arg-type]
    )
  for axis in AXES:
    await celigo.galvo._set_settling_window(
      axis,  # type: ignore[arg-type]
      POSITION_ERROR_WINDOW,
      VELOCITY_ERROR_WINDOW,
    )
  for axis in AXES:
    sweeps = []
    for _ in range(2):
      started = time.monotonic()
      passed = await celigo.galvo.calibrate(axis, timeout=CALIBRATION_TIMEOUT)  # type: ignore[arg-type]
      sweeps.append({"passed": passed, "seconds": round(time.monotonic() - started, 3)})
    result[f"{axis}_sweeps"] = sweeps
    result[f"{axis}_errors_after"] = _summarize_errors(
      await celigo.galvo.request_calibration_errors(axis)  # type: ignore[arg-type]
    )
  result["status_after"] = await _status(celigo)
  result["passed"] = all(sweep["passed"] for axis in AXES for sweep in result[f"{axis}_sweeps"])
  return result


async def move(celigo: Celigo, step: float, pause: float) -> List[Dict[str, Any]]:
  moves: List[Dict[str, Any]] = []
  for axis in AXES:
    for target in (step, -step, 0.0):
      started = time.monotonic()
      try:
        hardware = await celigo.galvo.move_single(axis, target)  # type: ignore[arg-type]
        outcome = "settled"
      except CeligoError as exc:
        hardware, outcome = target, f"FAILED: {exc}"
      status = await celigo.galvo.request_controller_status()
      moves.append(
        {
          "axis": axis,
          "target_volts": target,
          "outcome": outcome,
          "seconds": round(time.monotonic() - started, 3),
          "reported_volts": round(
            status.x_hardware_voltage if axis == "x" else status.y_hardware_voltage, 4
          ),
          "hardware_volts": hardware,
        }
      )
      await asyncio.sleep(pause)
  return moves


async def run(args: argparse.Namespace, report: Dict[str, Any]) -> None:
  celigo = make_celigo(args.port, reply_timeout=args.reply_timeout)
  galvo_config = draft_galvo_config(args.max_volts)
  celigo.config.hardware.x_galvo = galvo_config
  celigo.config.hardware.y_galvo = galvo_config
  try:
    await connect(celigo)
  except CeligoError as exc:
    report["handshake"] = f"FAILED: {exc}"
    return
  try:
    report["handshake"] = "ok"
    report["calibrate"] = await calibrate(celigo)
    if args.through == "move":
      if not report["calibrate"]["passed"]:
        raise CeligoError("calibration did not pass; skipping moves")
      report["moves"] = await move(celigo, args.step, args.pause)
  except CeligoError as exc:
    report["error"] = f"{type(exc).__name__}: {exc}"
  finally:
    await disconnect(celigo)


def main(argv: Optional[List[str]] = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument("--port", required=True, help="controller COM port, e.g. COM4")
  parser.add_argument("--through", choices=("calibrate", "move"), default="calibrate")
  parser.add_argument("--step", type=float, default=0.5, help="move test step in volts")
  parser.add_argument("--max-volts", type=float, default=1.0, help="draft galvo limit (+/-)")
  parser.add_argument("--pause", type=float, default=1.0, help="seconds between moves")
  parser.add_argument("--reply-timeout", type=float, default=5.0)
  parser.add_argument(
    "--out",
    type=Path,
    default=Path(__file__).resolve().parent / "runs",
    help="directory for the report and byte log",
  )
  args = parser.parse_args(argv)
  if not 0 < args.max_volts <= MAX_STEP_VOLTS:
    parser.error(f"--max-volts must be in (0, {MAX_STEP_VOLTS:g}]")
  if not 0 < args.step <= args.max_volts:
    parser.error("--step must be positive and within --max-volts")

  args.out.mkdir(parents=True, exist_ok=True)
  stamp = time.strftime("%Y%m%d-%H%M%S")
  name = f"celigo_galvo_{args.through}_{stamp}"
  logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s %(name)s %(message)s",
    handlers=[logging.FileHandler(args.out / f"{name}.log", encoding="utf-8")],
  )
  report: Dict[str, Any] = {
    "port": args.port,
    "through": args.through,
    "step_volts": args.step,
    "max_volts": args.max_volts,
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
