"""Light each analog (DAC) output of the legacy Celigo in turn, to find the lamp channels.

On newer instruments the four analog outputs set illumination intensity (brightfield and
fluorescence). This script drives one channel at a time to a low count, holds it so it can
be seen, reads it back, and returns it to 0, with a fixed schedule so each channel can be
identified by when it lights:

  channel n is on from about n * (on + off) seconds to that + on seconds

Digital outputs (lamp power, strobe, filter-select lines) are NOT touched. Every channel
in the run is forced back to 0 at the end, whatever happens.

  .venv\\Scripts\\python tools\\celigo_legacy\\illumination_test.py --port COM4
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

from pylabrobot.revvity.celigo import Celigo, CeligoError
from pylabrobot.revvity.celigo.protocol import complete_cleanup

DAC_FULL_SCALE = 4095
MAX_FRACTION = 0.25
ANALOG_CHANNELS = 4


async def _zero_channels(celigo: Celigo, channels: List[int]) -> Dict[int, Any]:
  """Set every channel to 0 and read it back; never raises."""
  readback: Dict[int, Any] = {}
  for channel in channels:
    try:
      await celigo.set_analog_output_count(channel, 0)
      readback[channel] = await celigo.request_analog_output_count(channel)
    except Exception as exc:  # keep zeroing the remaining channels
      readback[channel] = f"FAILED: {exc}"
  return readback


async def _analog_inputs(celigo: Celigo) -> List[Any]:
  values: List[Any] = []
  for channel in range(ANALOG_CHANNELS):
    try:
      values.append(await celigo.request_analog_input_count(channel))
    except CeligoError as exc:
      values.append(f"FAILED: {exc}")
  return values


async def light_channels(
  celigo: Celigo,
  channels: List[int],
  count: int,
  on_seconds: float,
  off_seconds: float,
  report: Dict[str, Any],
) -> None:
  results: List[Dict[str, Any]] = []
  report["channels"] = results
  started = time.monotonic()
  try:
    report["initial_zeroing"] = await _zero_channels(celigo, channels)
    for slot, channel in enumerate(channels):
      scheduled = slot * (on_seconds + off_seconds)
      await asyncio.sleep(max(0.0, scheduled - (time.monotonic() - started)))
      entry: Dict[str, Any] = {"channel": channel, "on_at": round(time.monotonic() - started, 2)}
      results.append(entry)
      print(f"t={entry['on_at']:5.1f}s  analog output {channel} ON (count {count})", flush=True)
      await celigo.set_analog_output_count(channel, count)
      entry["readback_on"] = await celigo.request_analog_output_count(channel)
      entry["analog_inputs_on"] = await _analog_inputs(celigo)
      await asyncio.sleep(on_seconds)
      await celigo.set_analog_output_count(channel, 0)
      entry["readback_off"] = await celigo.request_analog_output_count(channel)
      entry["off_at"] = round(time.monotonic() - started, 2)
      print(f"t={entry['off_at']:5.1f}s  analog output {channel} off", flush=True)
  finally:
    report["final_zeroing"] = await complete_cleanup(_zero_channels(celigo, channels))


async def run(args: argparse.Namespace, report: Dict[str, Any]) -> None:
  celigo = make_celigo(args.port, reply_timeout=args.reply_timeout)
  try:
    await connect(celigo)
  except CeligoError as exc:
    report["handshake"] = f"FAILED: {exc}"
    return
  try:
    report["handshake"] = "ok"
    report["digital_outputs_before"] = f"{await celigo.request_digital_output_bitmask():#06x}"
    await light_channels(
      celigo, args.channels, args.count, args.on_seconds, args.off_seconds, report
    )
  except CeligoError as exc:
    report["error"] = f"{type(exc).__name__}: {exc}"
  finally:
    with contextlib.suppress(Exception):
      await disconnect(celigo)


def main(argv: Optional[List[str]] = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument("--port", required=True, help="controller COM port, e.g. COM4")
  parser.add_argument(
    "--channels",
    type=int,
    nargs="+",
    default=list(range(ANALOG_CHANNELS)),
    choices=range(ANALOG_CHANNELS),
    help="analog outputs to light, in order (default 0 1 2 3)",
  )
  parser.add_argument(
    "--count",
    type=int,
    default=round(0.10 * DAC_FULL_SCALE),
    help=f"DAC count while on (default 10%% = 410, max {round(MAX_FRACTION * DAC_FULL_SCALE)})",
  )
  parser.add_argument("--on-seconds", type=float, default=5.0)
  parser.add_argument("--off-seconds", type=float, default=3.0)
  parser.add_argument("--reply-timeout", type=float, default=2.0)
  parser.add_argument(
    "--out",
    type=Path,
    default=Path(__file__).resolve().parent / "runs",
    help="directory for the report and byte log",
  )
  args = parser.parse_args(argv)
  if not 0 < args.count <= round(MAX_FRACTION * DAC_FULL_SCALE):
    parser.error(f"--count must be in 1..{round(MAX_FRACTION * DAC_FULL_SCALE)}")
  if len(set(args.channels)) != len(args.channels):
    parser.error("--channels must not repeat")

  args.out.mkdir(parents=True, exist_ok=True)
  stamp = time.strftime("%Y%m%d-%H%M%S")
  name = f"celigo_illumination_{stamp}"
  logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s %(name)s %(message)s",
    handlers=[logging.FileHandler(args.out / f"{name}.log", encoding="utf-8")],
  )
  report: Dict[str, Any] = {
    "port": args.port,
    "channels_requested": args.channels,
    "count": args.count,
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
