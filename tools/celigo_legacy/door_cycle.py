"""Open the door, take a plate, and close it, homing only when the axes need it.

Before connecting the driver, a query-only session checks X, Y, and Z:

* each motor's encoder ratio matches the configuration. The EZStepper drivers power up
  with ratio 1000, and only the driver's motor initialization sets the configured value,
  so a match means the controllers have not lost power (and their positions) since the
  driver configured them. The check cannot tell whether that session finished homing; the
  only way to configure the motors without homing is the stepwise test scripts here.
* each encoder position lies within the axis's configured travel, give or take the
  fine-position window that every move (including homing's last) settles within, the
  same rule ``LinearAxis.assume_homed`` applies.

If both hold, ``Celigo.setup(home=False)`` adopts the existing positions; otherwise the
full homing setup runs. Either way it then opens the door, waits for the plate, and closes
the door as ``door_demo.py`` does. It refuses to start while Y is out through the door.

  .venv\\Scripts\\python tools\\celigo_legacy\\door_cycle.py --port COM4
"""

import argparse
import asyncio
import logging
import math
import sys
import time
from pathlib import Path
from typing import List, Literal, Optional, Tuple

from controller import connect, disconnect, make_celigo
from door_demo import ensure_stage_inside, make_demo_celigo, run_demo
from legacy_config import load_legacy_config

from pylabrobot.revvity.celigo import LinearAxis, LinearAxisConfig

RATIO_TOLERANCE = 0.0005


async def homed_state(port: str) -> Tuple[bool, str]:
  """Whether X, Y, and Z still hold positions from an earlier homing, and why not."""
  hardware = load_legacy_config().hardware
  axes: List[Tuple[Literal["x", "y", "z"], Optional[LinearAxisConfig]]] = [
    ("x", hardware.x_axis),
    ("y", hardware.y_axis),
    ("z", hardware.z_axis),
  ]
  probe = make_celigo(port)
  await connect(probe)
  try:
    for name, config in axes:
      if config is None:
        return False, f"{name} is not configured"
      axis = LinearAxis(probe.motor_controller, name, config)
      ratio = await axis.request_encoder_ratio()
      if not math.isclose(ratio, config.encoder_to_motor_tick_ratio, abs_tol=RATIO_TOLERANCE):
        return False, (
          f"{name.upper()} motor has encoder ratio {ratio:g}, not the configured "
          f"{config.encoder_to_motor_tick_ratio:g}: it was power-cycled since the last setup"
        )
      ticks = await axis.request_encoder_ticks()
      low, high = axis.encoder_bounds()
      window = config.fine_position_error_window
      if not low - window <= ticks <= high + window:
        return False, (
          f"{name.upper()} is at {ticks} ticks, outside its travel {low}..{high} (+/- {window})"
        )
  finally:
    await disconnect(probe)
  return True, "X, Y, and Z hold their positions from the last homing"


async def cycle(port: str, plate_wait: Optional[float]) -> None:
  await ensure_stage_inside(port)
  homed, reason = await homed_state(port)
  print(f"{'Already homed' if homed else 'Homing needed'}: {reason}", flush=True)
  await run_demo(make_demo_celigo(port), plate_wait, home=not homed)


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
  log_path = args.out / f"celigo_door_cycle_{time.strftime('%Y%m%d-%H%M%S')}.log"
  logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s %(name)s %(message)s",
    handlers=[logging.FileHandler(log_path, encoding="utf-8")],
  )
  try:
    asyncio.run(cycle(args.port, args.plate_wait))
  except KeyboardInterrupt:
    print("Interrupted.")
    return 1
  print(f"byte log: {log_path}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
