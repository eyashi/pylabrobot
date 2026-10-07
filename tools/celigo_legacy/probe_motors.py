"""Step 2: read-only EZStepper motor queries for a legacy Celigo.

Every motor command sent here is a query (``&``, ``Q``, or ``?...``) without the ``R``
execute suffix, so no motor moves and no motor setting changes. Two modes:

* ``snapshot`` (default): per motor, read firmware, status, encoder position, encoder
  ratio, raw limit/input flags, and a few raw parameter queries.
* ``watch``: poll encoder positions and input flags and print every change, so a stage
  moved by hand can be matched to its motor address. Stop with Ctrl+C.

  .venv\\Scripts\\python tools\\celigo_legacy\\probe_motors.py --port COM4
  .venv\\Scripts\\python tools\\celigo_legacy\\probe_motors.py --port COM4 watch --seconds 120
"""

import argparse
import asyncio
import json
import logging
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from controller import connect, disconnect, make_celigo

from pylabrobot.revvity.celigo import Celigo, CeligoError, StepperMotor

# AllMotion queries whose meaning the Celigo driver does not depend on. Their replies are
# recorded verbatim; interpret them against the EZStepper manual for the motor firmware.
RAW_QUERIES: Tuple[str, ...] = ("?0", "?1", "?2", "?6", "?s0")


def _is_query(command_tokens: str) -> bool:
  return command_tokens in ("&", "Q") or command_tokens.startswith("?")


async def _query(motor: StepperMotor, command_tokens: str) -> Dict[str, Any]:
  """Send one query and return the parsed reply, never raising for a device error."""
  if not _is_query(command_tokens):
    raise ValueError(f"{command_tokens!r} is not a query")
  try:
    response = await motor.send_command(command_tokens, execute=False)
  except CeligoError as exc:
    return {"error": str(exc)}
  return {
    "ready": response.ready,
    "error_code": response.error_code,
    "text": response.response_text,
  }


async def snapshot_motor(motor: StepperMotor) -> Dict[str, Any]:
  result: Dict[str, Any] = {}
  for name, request in (
    ("firmware", motor.request_motor_controller_firmware_version),
    ("encoder_ticks", motor.request_encoder_ticks),
    ("encoder_ratio", motor.request_encoder_ratio),
  ):
    try:
      result[name] = await request()
    except (CeligoError, ValueError) as exc:
      result[name] = f"FAILED: {exc}"
  result["firmware_text"] = (await _query(motor, "&")).get("text")
  result["status"] = await _query(motor, "Q")
  # Raw input flags; the driver applies per-axis polarity, which is not known yet.
  result["input_flags"] = await _query(motor, "?4")
  result["raw_queries"] = {tokens: await _query(motor, tokens) for tokens in RAW_QUERIES}
  return result


async def _read_position(motor: StepperMotor) -> Tuple[Optional[int], Optional[str]]:
  try:
    ticks: Optional[int] = await motor.request_encoder_ticks()
  except (CeligoError, ValueError):
    ticks = None
  flags = await _query(motor, "?4")
  return ticks, flags.get("text", flags.get("error"))


async def watch(
  celigo: Celigo,
  addresses: Sequence[int],
  seconds: float,
  interval: float,
) -> List[Dict[str, Any]]:
  """Poll encoders and input flags; return and print every observed change."""
  motors = {address: StepperMotor(celigo.motor_controller, address) for address in addresses}
  last: Dict[int, Tuple[Optional[int], Optional[str]]] = {}
  changes: List[Dict[str, Any]] = []
  deadline = time.monotonic() + seconds
  try:
    while time.monotonic() < deadline:
      for address, motor in motors.items():
        current = await _read_position(motor)
        previous = last.get(address)
        if current != previous:
          change = {
            "t": round(seconds - (deadline - time.monotonic()), 2),
            "address": address,
            "encoder_ticks": current[0],
            "input_flags": current[1],
            "delta_ticks": (
              None
              if previous is None or previous[0] is None or current[0] is None
              else current[0] - previous[0]
            ),
          }
          changes.append(change)
          print(json.dumps(change), flush=True)
          last[address] = current
      await asyncio.sleep(interval)
  except (KeyboardInterrupt, asyncio.CancelledError):
    pass
  return changes


async def run(args: argparse.Namespace, report: Dict[str, Any]) -> None:
  """Fill ``report`` in place, so a Ctrl+C still leaves the partial results."""
  celigo = make_celigo(args.port, reply_timeout=args.reply_timeout)
  try:
    await connect(celigo)
  except CeligoError as exc:
    report["handshake"] = f"FAILED: {exc}"
    return
  try:
    report["handshake"] = "ok"
    addresses = args.addresses or sorted(
      motor.motor_index for motor in await celigo.request_detected_motor_addresses()
    )
    report["addresses"] = addresses
    if args.mode == "watch":
      print(f"Watching motors {addresses} for {args.seconds:g} s; Ctrl+C stops early.")
      report["changes"] = await watch(celigo, addresses, args.seconds, args.interval)
    else:
      report["motors"] = {
        str(address): await snapshot_motor(StepperMotor(celigo.motor_controller, address))
        for address in addresses
      }
  finally:
    await disconnect(celigo)


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument("mode", nargs="?", choices=("snapshot", "watch"), default="snapshot")
  parser.add_argument("--port", required=True, help="controller COM port, e.g. COM4")
  parser.add_argument(
    "--addresses",
    type=int,
    nargs="+",
    help="motor addresses to query (default: every detected motor)",
  )
  parser.add_argument("--seconds", type=float, default=60.0, help="watch duration")
  parser.add_argument("--interval", type=float, default=0.2, help="watch poll interval")
  parser.add_argument("--reply-timeout", type=float, default=2.0)
  parser.add_argument(
    "--out",
    type=Path,
    default=Path(__file__).resolve().parent / "runs",
    help="directory for the report and byte log",
  )
  args = parser.parse_args()

  args.out.mkdir(parents=True, exist_ok=True)
  stamp = time.strftime("%Y%m%d-%H%M%S")
  log_path = args.out / f"celigo_motors_{args.mode}_{stamp}.log"
  logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s %(name)s %(message)s",
    handlers=[logging.FileHandler(log_path, encoding="utf-8")],
  )

  report: Dict[str, Any] = {
    "port": args.port,
    "mode": args.mode,
    "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
  }
  try:
    asyncio.run(run(args, report))
  except KeyboardInterrupt:
    report["interrupted"] = True
  report_path = args.out / f"celigo_motors_{args.mode}_{stamp}.json"
  report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
  if args.mode == "snapshot":
    print(json.dumps(report, indent=2))
  print(f"\nreport: {report_path}\nbyte log: {log_path}")
  return 0 if report.get("handshake") == "ok" else 1


if __name__ == "__main__":
  sys.exit(main())
