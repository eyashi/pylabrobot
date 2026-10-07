"""Door demonstration: home the legacy Celigo, present the stage, take a plate, close.

Runs the unmodified PyLabRobot driver flow on the legacy configuration
(``legacy_config.load_legacy_config``) without a camera:

0. Refuse to start if Y is out through the door (its front sensor, bit 3, is active):
   ``home_imaging_axes()`` homes X before Y, which would push an extended stage sideways
   into the door frame.
1. ``Celigo.setup()``: handshake, motor and galvo initialization, then
   ``home_imaging_axes()`` (Z down to its sensor, X, Y, then the filter wheel).
2. ``Celigo.open_drawer()``: Z down, Y to the rear, X to the door, Y out to the front.
3. Wait for the plate (Enter, or ``--plate-wait`` seconds).
4. ``Celigo.close_drawer_to_sample_mm(0, 0)``: Y in to the rear first, then X and Y to
   the park pose just off their home sensors (not yet calibrated against the optics).
5. ``Celigo.stop()``: outputs off, transport closed. Always runs.

  .venv\\Scripts\\python tools\\celigo_legacy\\door_demo.py --port COM4
"""

import argparse
import asyncio
import logging
import sys
import time
from pathlib import Path
from typing import Callable, List, Optional

from controller import connect, disconnect, make_celigo
from legacy_config import load_legacy_config
from serial_io import CeligoSerialIO

from pylabrobot.revvity.celigo import Celigo, CeligoError, StepperMotor

# Homing Y from the door (~250 mm at the 1 mm/s homing speed) exceeds the driver's 30 s
# default for the moves that follow it.
MOVE_TIMEOUT = 180.0
Y_AXIS_ADDRESS = 2
FAR_SENSOR_BIT = 0x08


def make_demo_celigo(port: str) -> Celigo:
  """The driver's Celigo on the legacy config, without a camera, over the COM-port adapter."""
  celigo = Celigo(config=load_legacy_config(), camera_enabled=False, move_timeout=MOVE_TIMEOUT)
  celigo.io = CeligoSerialIO(port, baudrate=celigo.baudrate)  # type: ignore[assignment]
  return celigo


async def ensure_stage_inside(port: str) -> None:
  """Read Y's sensors over a short query-only session; refuse if the stage is out."""
  probe = make_celigo(port)
  await connect(probe)
  try:
    response = await StepperMotor(probe.motor_controller, Y_AXIS_ADDRESS).send_command(
      "?4", execute=False
    )
  finally:
    await disconnect(probe)
  if int(response.response_text) & FAR_SENSOR_BIT:
    raise CeligoError(
      "Y is out through the door (front sensor active). Bring it back inside before running "
      "the demo: setup() homes X before Y."
    )


async def report_positions(celigo: Celigo, label: str) -> None:
  x = await celigo.x_axis.request_position()
  y = await celigo.y_axis.request_position()
  z = await celigo.z_axis.request_position()
  print(f"{label}: X {x:.2f} mm, Y {y:.2f} mm, Z {z:.3f} mm", flush=True)


async def wait_for_plate(plate_wait: Optional[float], prompt: Callable[[str], str]) -> None:
  if plate_wait is not None:
    print(f"Place the plate now; closing in {plate_wait:g} s.", flush=True)
    await asyncio.sleep(plate_wait)
    return
  await asyncio.get_running_loop().run_in_executor(
    None, prompt, "Place the plate on the stage, then press Enter to close the door... "
  )


async def run_demo(
  celigo: Celigo,
  plate_wait: Optional[float],
  prompt: Callable[[str], str] = input,
  home: bool = True,
) -> None:
  """Steps 1-5 of the module docstring; the caller runs ``ensure_stage_inside`` first.

  ``home=False`` passes through to ``Celigo.setup`` to reuse the positions of an earlier
  homing instead of homing again.
  """
  try:
    if home:
      print("Setting up and homing (Z, X, Y, filter wheel)...", flush=True)
    else:
      print("Setting up without homing (reusing the earlier homing)...", flush=True)
    started = time.monotonic()
    await celigo.setup(home=home)
    print(f"Setup took {time.monotonic() - started:.0f} s", flush=True)
    await report_positions(celigo, "Ready")

    print("Opening the door (presenting the stage)...", flush=True)
    await celigo.open_drawer()
    await report_positions(celigo, "Door open")

    await wait_for_plate(plate_wait, prompt)

    print("Closing the door (Y in first, then parking near the home sensors)...", flush=True)
    await celigo.close_drawer_to_sample_mm(0.0, 0.0)
    await report_positions(celigo, "Door closed")
  finally:
    await celigo.stop()
    print("Stopped: outputs off, connection closed.", flush=True)


def main(argv: Optional[List[str]] = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument("--port", required=True, help="controller COM port, e.g. COM4")
  parser.add_argument(
    "--plate-wait",
    type=float,
    help="seconds to wait with the door open instead of waiting for Enter",
  )
  parser.add_argument(
    "--out",
    type=Path,
    default=Path(__file__).resolve().parent / "runs",
    help="directory for the byte log",
  )
  args = parser.parse_args(argv)

  args.out.mkdir(parents=True, exist_ok=True)
  log_path = args.out / f"celigo_door_demo_{time.strftime('%Y%m%d-%H%M%S')}.log"
  logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s %(name)s %(message)s",
    handlers=[logging.FileHandler(log_path, encoding="utf-8")],
  )

  async def demo() -> None:
    await ensure_stage_inside(args.port)
    await run_demo(make_demo_celigo(args.port), args.plate_wait)

  try:
    asyncio.run(demo())
  except KeyboardInterrupt:
    print("Interrupted.")
    return 1
  print(f"byte log: {log_path}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
