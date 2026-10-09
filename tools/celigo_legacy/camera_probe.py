"""Bring up the Allied Vision FireWire camera with the driver's ``AlliedVisionCamera``.

Runs on the PC that holds the FireWire card; no motor controller is needed. Each stage
opens the camera, does its work, closes it, and writes ``runs/celigo_camera_<stage>_*.json``
plus any images. Nothing here moves or lights anything.

  info     SDK version, camera identity, setup notes, geometry, and every feature's value.
  capture  One frame per ``--exposures-ms`` value: timing, statistics, unused low bits
           (bit alignment of Mono16), and a downsampled PNG for viewing.
  roi      Sets a centered ROI with ``set_frame_format`` and captures one frame in it.

  .venv\\Scripts\\python tools\\celigo_legacy\\camera_probe.py info
  .venv\\Scripts\\python tools\\celigo_legacy\\camera_probe.py capture --exposures-ms 1 10 100
  .venv\\Scripts\\python tools\\celigo_legacy\\camera_probe.py roi --width 1024 --height 1024
"""

import argparse
import asyncio
import contextlib
import dataclasses
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from pylabrobot.revvity.celigo.allied_vision_camera import AlliedVisionCamera
from pylabrobot.revvity.celigo.camera import CameraError, CameraFrame


def make_camera(args: argparse.Namespace) -> AlliedVisionCamera:
  """The camera under test, as configured on the command line."""
  return AlliedVisionCamera(
    camera_id=args.camera_id,
    pixel_format=args.pixel_format,
    sdk_library=args.sdk_library,
  )


def unused_low_bits(frame: CameraFrame) -> int:
  """Number of low bits that are zero in every pixel (2 for 14-bit data MSB-aligned in 16)."""
  combined = 0
  for value in frame.pixels():
    combined |= value
  if combined == 0:
    return frame.bit_depth
  count = 0
  while not combined & 1:
    combined >>= 1
    count += 1
  return count


def describe_camera(camera: AlliedVisionCamera) -> Dict[str, Any]:
  """Identity and current state of an open camera."""
  return {
    "sdk_version": camera.sdk_version,
    "camera": dataclasses.asdict(camera.info) if camera.info else None,
    "setup_notes": camera.setup_notes,
    "geometry": {
      "width": camera.width,
      "height": camera.height,
      "x_offset": camera.x_offset,
      "y_offset": camera.y_offset,
      "bit_depth": camera.bit_depth,
    },
    "exposure_feature": camera.exposure_feature,
    "exposure_ms": camera.exposure_ms,
    "gain_feature": camera.gain_feature,
    "gain": camera.gain,
  }


async def capture_and_record(
  camera: AlliedVisionCamera, out: Path, name: str, png_size: int
) -> Dict[str, Any]:
  """Capture one frame without flushing; save a PNG; return its timing and statistics."""
  started = time.perf_counter()
  frame = await camera.capture(flush_frames=0)
  elapsed = time.perf_counter() - started
  minimum, maximum, mean = frame.statistics()
  png = out / f"{name}.png"
  frame.save_png(str(png), maximum_size=png_size)
  return {
    "exposure_ms": frame.exposure_ms,
    "gain": frame.gain,
    "elapsed_s": round(elapsed, 4),
    "size": [frame.width, frame.height, frame.bit_depth],
    "min": minimum,
    "max": maximum,
    "mean": round(mean, 2),
    "unused_low_bits": unused_low_bits(frame),
    "png": str(png),
  }


async def run_info(camera: AlliedVisionCamera, args: argparse.Namespace, report: Dict[str, Any]):
  report.update(describe_camera(camera))
  report["features"] = [dataclasses.asdict(feature) for feature in await camera.list_features()]


async def run_capture(camera: AlliedVisionCamera, args: argparse.Namespace, report: Dict[str, Any]):
  report.update(describe_camera(camera))
  if args.gain is not None:
    report["gain_applied"] = await camera.set_gain(args.gain)
  frames: List[Dict[str, Any]] = []
  report["frames"] = frames
  for exposure in args.exposures_ms:
    applied = await camera.set_exposure(exposure)
    record = await capture_and_record(
      camera, args.out, f"{args.name}_{exposure:g}ms", args.png_size
    )
    record["requested_exposure_ms"] = exposure
    record["applied_exposure_ms"] = applied
    frames.append(record)


async def run_roi(camera: AlliedVisionCamera, args: argparse.Namespace, report: Dict[str, Any]):
  report["full_frame"] = [camera.width, camera.height]
  report["set_frame_format"] = await camera.set_frame_format(args.width, args.height)
  report.update(describe_camera(camera))
  report["frame"] = await capture_and_record(camera, args.out, f"{args.name}_roi", args.png_size)


STAGES = {"info": run_info, "capture": run_capture, "roi": run_roi}


async def run(args: argparse.Namespace, report: Dict[str, Any]) -> None:
  camera = make_camera(args)
  try:
    await camera.setup()
    report["setup"] = "ok"
    await STAGES[args.stage](camera, args, report)
  except (CameraError, ValueError) as exc:
    report["error"] = f"{type(exc).__name__}: {exc}"
  finally:
    with contextlib.suppress(Exception):
      await camera.stop()


def main(argv: Optional[List[str]] = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument("stage", choices=sorted(STAGES))
  parser.add_argument("--camera-id", help="Vimba camera ID; the first camera when omitted")
  parser.add_argument("--pixel-format", choices=["Mono8", "Mono16"], default="Mono16")
  parser.add_argument("--sdk-library", help="path to the 64-bit VimbaC.dll")
  parser.add_argument("--exposures-ms", type=float, nargs="+", default=[1.0, 10.0, 100.0])
  parser.add_argument("--gain", type=float, help="native gain to apply before capturing")
  parser.add_argument("--width", type=int, default=1024)
  parser.add_argument("--height", type=int, default=1024)
  parser.add_argument("--png-size", type=int, default=1024, help="longest PNG side in pixels")
  parser.add_argument(
    "--out",
    type=Path,
    default=Path(__file__).resolve().parent / "runs",
    help="directory for reports and images",
  )
  args = parser.parse_args(argv)
  args.out.mkdir(parents=True, exist_ok=True)
  args.name = f"celigo_camera_{args.stage}_{time.strftime('%Y%m%d-%H%M%S')}"
  report: Dict[str, Any] = {
    "stage": args.stage,
    "pixel_format": args.pixel_format,
    "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
  }
  try:
    asyncio.run(run(args, report))
  except KeyboardInterrupt:
    report["interrupted"] = True
  report_path = args.out / f"{args.name}.json"
  report_path.write_text(json.dumps(report, indent=2, default=str), encoding="utf-8")
  summary = {key: value for key, value in report.items() if key != "features"}
  print(json.dumps(summary, indent=2, default=str))
  print(f"\nreport: {report_path}")
  return 0 if report.get("setup") == "ok" and "error" not in report else 1


if __name__ == "__main__":
  sys.exit(main())
