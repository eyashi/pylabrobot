"""Step 1: read-only controller probe for a legacy Celigo.

Opens the controller's COM port, performs the binary handshake, and reports controller
status, identity, detected motor addresses, and digital inputs. It sends no motor
commands and changes no outputs. Every byte exchanged is logged to the log file.

  .venv\\Scripts\\python tools\\celigo_legacy\\probe_controller.py --port COM4
"""

import argparse
import asyncio
import json
import logging
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any, Dict

from controller import connect, disconnect, make_celigo

from pylabrobot.revvity.celigo import CeligoError
from pylabrobot.revvity.celigo.motion import (
  _LENGTH_PREFIXED_COMMAND_MINIMUM_FIRMWARE as LENGTH_PREFIXED_MINIMUM_FIRMWARE,
)


async def probe(port: str, reply_timeout: float) -> Dict[str, Any]:
  report: Dict[str, Any] = {"port": port, "started_at": time.strftime("%Y-%m-%dT%H:%M:%S")}
  celigo = make_celigo(port, reply_timeout=reply_timeout)
  try:
    status = await connect(celigo)
  except CeligoError as exc:
    report["handshake"] = f"FAILED: {exc}"
    return report
  try:
    report["handshake"] = "ok"
    report["controller_status"] = {
      **asdict(status),
      "busy": status.busy,
      "error": status.error,
      "interlock_open": status.interlock_open,
      "controller_failed": status.controller_failed,
    }
    info = celigo.controller_info
    assert info is not None
    report["controller_info"] = asdict(info)
    report["motor_command_framing"] = (
      "length-prefixed OEM frames (opcode 47)"
      if info.firmware_version >= LENGTH_PREFIXED_MINIMUM_FIRMWARE
      else "raw ASCII (opcode 44, legacy path)"
    )
    try:
      motors = await celigo.request_detected_motor_addresses()
      report["detected_motors"] = [asdict(motor) for motor in motors]
    except CeligoError as exc:
      report["detected_motors"] = f"FAILED: {exc}"
    try:
      report["digital_inputs"] = f"{await celigo.request_digital_input_bitmask():#06x}"
    except CeligoError as exc:
      report["digital_inputs"] = f"FAILED: {exc}"
  finally:
    await disconnect(celigo)
  return report


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument("--port", required=True, help="controller COM port, e.g. COM4")
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
  log_path = args.out / f"celigo_probe_{stamp}.log"
  logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s %(name)s %(message)s",
    handlers=[logging.FileHandler(log_path, encoding="utf-8")],
  )

  report = asyncio.run(probe(args.port, args.reply_timeout))
  report_path = args.out / f"celigo_probe_{stamp}.json"
  report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
  print(json.dumps(report, indent=2))
  print(f"\nreport: {report_path}\nbyte log: {log_path}")
  return 0 if report.get("handshake") == "ok" else 1


if __name__ == "__main__":
  sys.exit(main())
