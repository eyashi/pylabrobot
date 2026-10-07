"""Read-only survey of the legacy Celigo controller's I/O.

Every command here is a read: nothing moves, and no output, galvo, camera trigger, barcode
display, or laser state is changed. Two modes:

* ``snapshot`` (default): read controller status and identity, digital inputs/outputs,
  analog inputs/outputs, galvo status and calibration data, camera signal lines, the
  barcode buffer, and every motor's sensor inputs. Each read is independent, so hardware
  this unit lacks shows up as a failed entry rather than stopping the survey.
* ``watch``: poll the inputs and status flags and print every change, so switches (door,
  lid, drawer, interlock) can be matched to the bits that report them.

  .venv\\Scripts\\python tools\\celigo_legacy\\io_survey.py --port COM4
  .venv\\Scripts\\python tools\\celigo_legacy\\io_survey.py --port COM4 watch --seconds 120
"""

import argparse
import asyncio
import json
import logging
import sys
import time
from dataclasses import asdict, is_dataclass
from functools import partial
from pathlib import Path
from typing import Any, Awaitable, Callable, Dict, List, Optional

from controller import connect, disconnect, make_celigo

from pylabrobot.revvity.celigo import Celigo, CeligoError, StepperMotor

# SIGNAL_DIAGNOSTICS read sub-commands (celigo.py _DIAG_READ_*); 1-3 drive the trigger line.
CAMERA_SIGNAL_READS = {"camera_busy": 4, "camera_integration": 5, "camera_trigger_encoder": 6}
DIGITAL_BITS = 12
ANALOG_CHANNELS = 4


def _plain(value: Any) -> Any:
  if is_dataclass(value) and not isinstance(value, type):
    return asdict(value)
  if isinstance(value, list):
    return [_plain(item) for item in value]
  return value


def _bits(mask: int) -> List[int]:
  return [bit for bit in range(DIGITAL_BITS) if mask & (1 << bit)]


async def _record(results: Dict[str, Any], name: str, read: Callable[[], Awaitable[Any]]) -> None:
  try:
    results[name] = _plain(await read())
  except (CeligoError, ValueError) as exc:
    results[name] = f"FAILED: {exc}"


async def _motor_input_flags(celigo: Celigo, address: int) -> int:
  response = await StepperMotor(celigo.motor_controller, address).send_command("?4", execute=False)
  if not response.ok:
    raise CeligoError(f"motor {address} ?4 failed (code {response.error_code})")
  return int(response.response_text)


async def snapshot(celigo: Celigo, motor_addresses: List[int]) -> Dict[str, Any]:
  results: Dict[str, Any] = {}

  async def status() -> Dict[str, Any]:
    value = await celigo.request_controller_status()
    return {
      **asdict(value),
      "busy": value.busy,
      "error": value.error,
      "interlock_open": value.interlock_open,
      "controller_failed": value.controller_failed,
    }

  async def digital_inputs() -> Dict[str, Any]:
    mask = await celigo.request_digital_input_bitmask()
    return {"mask": f"{mask:#06x}", "set_bits": _bits(mask)}

  async def digital_outputs() -> Dict[str, Any]:
    mask = await celigo.request_digital_output_bitmask()
    return {"mask": f"{mask:#06x}", "set_bits": _bits(mask)}

  await _record(results, "controller_status", status)
  await _record(results, "controller_info", celigo.request_controller_info)
  await _record(results, "digital_inputs", digital_inputs)
  await _record(results, "digital_outputs", digital_outputs)
  for channel in range(ANALOG_CHANNELS):
    await _record(
      results, f"analog_input_{channel}", partial(celigo.request_analog_input_count, channel)
    )
    await _record(
      results, f"analog_output_{channel}", partial(celigo.request_analog_output_count, channel)
    )
  await _record(results, "galvo_status", celigo.galvo.request_controller_status)
  await _record(
    results, "galvo_x_calibration_errors", partial(celigo.galvo.request_calibration_errors, "x")
  )
  await _record(
    results, "galvo_y_calibration_errors", partial(celigo.galvo.request_calibration_errors, "y")
  )
  await _record(results, "galvo_position_trace", celigo.galvo.request_position_trace_dac_counts)
  for name, operation in CAMERA_SIGNAL_READS.items():
    await _record(results, name, partial(celigo._send_signal_diagnostic_command, operation))
  await _record(results, "barcode_buffer", celigo.request_barcode)
  for address in motor_addresses:
    await _record(
      results, f"motor_{address}_input_flags", partial(_motor_input_flags, celigo, address)
    )
  return results


async def read_watched(celigo: Celigo, motor_addresses: List[int]) -> Dict[str, Any]:
  """The values watch mode compares between polls."""
  values: Dict[str, Any] = {}
  status = await celigo.request_controller_status()
  values["status_flags"] = status.raw_flags
  values["extended_status"] = status.extended_status
  values["digital_inputs"] = await celigo.request_digital_input_bitmask()
  for channel in range(ANALOG_CHANNELS):
    values[f"analog_input_{channel}"] = await celigo.request_analog_input_count(channel)
  for address in motor_addresses:
    values[f"motor_{address}_input_flags"] = await _motor_input_flags(celigo, address)
  return values


def _changed(name: str, old: Any, new: Any, analog_threshold: int) -> bool:
  if name.startswith("analog_input_"):
    return bool(abs(new - old) >= analog_threshold)
  return bool(new != old)


async def watch(
  celigo: Celigo,
  motor_addresses: List[int],
  seconds: float,
  interval: float,
  analog_threshold: int,
) -> List[Dict[str, Any]]:
  started = time.monotonic()
  baseline = await read_watched(celigo, motor_addresses)
  changes: List[Dict[str, Any]] = [{"t": 0.0, "baseline": baseline}]
  print(json.dumps(changes[0]), flush=True)
  try:
    while time.monotonic() - started < seconds:
      await asyncio.sleep(interval)
      current = await read_watched(celigo, motor_addresses)
      for name, value in current.items():
        if _changed(name, baseline[name], value, analog_threshold):
          change = {
            "t": round(time.monotonic() - started, 2),
            "signal": name,
            "from": baseline[name],
            "to": value,
          }
          if name == "digital_inputs":
            change["bits_toggled"] = _bits(baseline[name] ^ value)
          changes.append(change)
          print(json.dumps(change), flush=True)
          baseline[name] = value
  except (KeyboardInterrupt, asyncio.CancelledError):
    pass
  return changes


async def run(args: argparse.Namespace, report: Dict[str, Any]) -> None:
  celigo = make_celigo(args.port, reply_timeout=args.reply_timeout)
  try:
    await connect(celigo)
  except CeligoError as exc:
    report["handshake"] = f"FAILED: {exc}"
    return
  try:
    report["handshake"] = "ok"
    motors = sorted(motor.motor_index for motor in await celigo.request_detected_motor_addresses())
    report["motor_addresses"] = motors
    if args.mode == "watch":
      print(f"Watching for {args.seconds:g} s; operate the door, lid, and switches now.")
      report["changes"] = await watch(
        celigo, motors, args.seconds, args.interval, args.analog_threshold
      )
    else:
      report["snapshot"] = await snapshot(celigo, motors)
  finally:
    await disconnect(celigo)


def main(argv: Optional[List[str]] = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument("mode", nargs="?", choices=("snapshot", "watch"), default="snapshot")
  parser.add_argument("--port", required=True, help="controller COM port, e.g. COM4")
  parser.add_argument("--seconds", type=float, default=60.0, help="watch duration")
  parser.add_argument("--interval", type=float, default=0.2, help="watch poll interval")
  parser.add_argument(
    "--analog-threshold", type=int, default=20, help="analog change (counts) worth reporting"
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
  name = f"celigo_io_{args.mode}_{stamp}"
  logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s %(name)s %(message)s",
    handlers=[logging.FileHandler(args.out / f"{name}.log", encoding="utf-8")],
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
  report_path = args.out / f"{name}.json"
  report_path.write_text(json.dumps(report, indent=2, default=str), encoding="utf-8")
  if args.mode == "snapshot":
    print(json.dumps(report, indent=2, default=str))
  print(f"\nreport: {report_path}\nbyte log: {args.out / f'{name}.log'}")
  return 0 if report.get("handshake") == "ok" else 1


if __name__ == "__main__":
  sys.exit(main())
