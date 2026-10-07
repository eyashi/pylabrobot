"""Turn on brightfield through the driver's own API, with pylabrobot's lighting I/O map.

Uses the I/O assignments from the driver's test fixtures
(pylabrobot/revvity/celigo/tests/device_configuration_tests.py): brightfield intensity on
analog output 0 mapped 0-10 V, the fluorescence strobe ``FLOnOff`` on digital bit 6.

For each requested intensity it calls ``Celigo.set_brightfield_enabled(True)`` (strobe off,
then analog 0 at that intensity), holds, then ``set_brightfield_enabled(False)``. The run
always ends with ``Celigo.turn_off_illumination()``. Lamp power, filter-select lines,
galvos, motors, and the laser are not touched.

  .venv\\Scripts\\python tools\\celigo_legacy\\brightfield_test.py --port COM4
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

from pylabrobot.revvity.celigo import (
  Celigo,
  CeligoError,
  DigitalIOConfig,
  IlluminationChannelConfig,
  IOConfig,
  LightingIOConfig,
)
from pylabrobot.revvity.celigo.protocol import complete_cleanup

MAX_PERCENT = 80.0
BRIGHTFIELD_ANALOG_CHANNEL = 0
STROBE_BIT = 6


def fixture_io_config() -> IOConfig:
  """Lighting map from the driver's test fixtures; lamp power and filter bits omitted."""
  return IOConfig(
    analog_ins=[],
    lighting_ios=[
      LightingIOConfig(
        config_version=1000,
        controller_index=0,
        channel=BRIGHTFIELD_ANALOG_CHANNEL,
        enabled=True,
        invert=False,
        io_name="eBrightFieldIntensity",
        min_voltage=0,
        max_voltage=10,
        delay=0.0,
      ),
    ],
    digital_ios=[
      DigitalIOConfig(
        config_version=1000,
        io_type="Out",
        bit_index=STROBE_BIT,
        invert=False,
        enabled=True,
        io_name="FLOnOff",
      ),
    ],
  )


def brightfield_channel(intensity_percent: float) -> IlluminationChannelConfig:
  return IlluminationChannelConfig(
    name="brightfield",
    display_name="Brightfield",
    logical_filter=1,
    bit_value=None,
    intensity_percent=intensity_percent,
    lighting_io_name="eBrightFieldIntensity",
    strobe=False,
    z_offset_to_brightfield_mm=0.0,
    mm_per_pixel_x_correction_to_brightfield=1.0,
    mm_per_pixel_y_correction_to_brightfield=1.0,
  )


async def _outputs(celigo: Celigo) -> Dict[str, Any]:
  return {
    "analog_0_count": await celigo.request_analog_output_count(BRIGHTFIELD_ANALOG_CHANNEL),
    "digital_outputs": f"{await celigo.request_digital_output_bitmask():#06x}",
  }


async def step_through(
  celigo: Celigo,
  percents: List[float],
  on_seconds: float,
  off_seconds: float,
  report: Dict[str, Any],
) -> None:
  steps: List[Dict[str, Any]] = []
  report["steps"] = steps
  started = time.monotonic()
  try:
    for percent in percents:
      celigo.config.channels["brightfield"] = brightfield_channel(percent)
      on_at = round(time.monotonic() - started, 2)
      print(f"t={on_at:5.1f}s  brightfield ON at {percent:g}%", flush=True)
      await celigo.set_brightfield_enabled(True)
      step: Dict[str, Any] = {"percent": percent, "on_at": on_at, "on": await _outputs(celigo)}
      steps.append(step)
      await asyncio.sleep(on_seconds)
      await celigo.set_brightfield_enabled(False)
      step["off"] = await _outputs(celigo)
      off_at = round(time.monotonic() - started, 2)
      print(f"t={off_at:5.1f}s  brightfield off", flush=True)
      await asyncio.sleep(off_seconds)
  finally:
    with contextlib.suppress(Exception):
      await complete_cleanup(celigo.turn_off_illumination())
    with contextlib.suppress(Exception):
      report["final_outputs"] = await complete_cleanup(_outputs(celigo))


async def run(args: argparse.Namespace, report: Dict[str, Any]) -> None:
  celigo = make_celigo(args.port, reply_timeout=args.reply_timeout)
  celigo.config.hardware.io = fixture_io_config()
  celigo.config.channels_by_magnification[celigo.config.magnification] = {
    "brightfield": brightfield_channel(args.percents[0])
  }
  try:
    await connect(celigo)
  except CeligoError as exc:
    report["handshake"] = f"FAILED: {exc}"
    return
  try:
    report["handshake"] = "ok"
    report["initial_outputs"] = await _outputs(celigo)
    await step_through(celigo, args.percents, args.on_seconds, args.off_seconds, report)
  except CeligoError as exc:
    report["error"] = f"{type(exc).__name__}: {exc}"
  finally:
    with contextlib.suppress(Exception):
      await disconnect(celigo)


def main(argv: Optional[List[str]] = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument("--port", required=True, help="controller COM port, e.g. COM4")
  parser.add_argument(
    "--percents",
    type=float,
    nargs="+",
    default=[25.0, 50.0, 70.0],
    help=f"brightfield intensities to step through (max {MAX_PERCENT:g})",
  )
  parser.add_argument("--on-seconds", type=float, default=4.0)
  parser.add_argument("--off-seconds", type=float, default=3.0)
  parser.add_argument("--reply-timeout", type=float, default=2.0)
  parser.add_argument(
    "--out",
    type=Path,
    default=Path(__file__).resolve().parent / "runs",
    help="directory for the report and byte log",
  )
  args = parser.parse_args(argv)
  if not all(0 < percent <= MAX_PERCENT for percent in args.percents):
    parser.error(f"--percents must each be in (0, {MAX_PERCENT:g}]")

  args.out.mkdir(parents=True, exist_ok=True)
  stamp = time.strftime("%Y%m%d-%H%M%S")
  name = f"celigo_brightfield_{stamp}"
  logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s %(name)s %(message)s",
    handlers=[logging.FileHandler(args.out / f"{name}.log", encoding="utf-8")],
  )
  report: Dict[str, Any] = {
    "port": args.port,
    "percents": args.percents,
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
