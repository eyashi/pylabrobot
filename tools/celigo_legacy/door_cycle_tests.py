"""Offline tests for the door cycle's homed check and flow choice."""

import asyncio
import struct
import unittest
from typing import Any, Dict, List
from unittest.mock import patch

import door_cycle
import serial_io
from probe_motors_tests import FakeMotorPort, _oem_reply


class FakeAxesPort(FakeMotorPort):
  """X, Y, Z as left by a homing run: configured ratios, parked 1 mm / 0.5 mm off home."""

  def __init__(self) -> None:
    super().__init__()
    self.ratios = {1: 256000, 2: 256000, 3: 32000, 4: 25600}
    self.positions = {1: 79, 2: 79, 3: 500, 4: 0}

  def _payload(self, opcode: int) -> bytes:
    if opcode != 47:
      return super()._payload(opcode)
    request = self._last_request
    address = int(chr(request[1]))
    tokens = request[3:-2].decode("ascii")
    texts: Dict[str, str] = {
      "?aE": str(self.ratios[address]),
      "?8": str(self.positions[address]),
      "?4": "3",
    }
    reply = _oem_reply(texts.get(tokens, ""))
    return struct.pack(">HH", 0, len(reply)) + reply


class DoorCycleTests(unittest.TestCase):
  def homed_state(self, port: FakeAxesPort) -> Any:
    with patch.object(serial_io.serial, "serial_for_url", return_value=port):
      return asyncio.run(door_cycle.homed_state("COM_TEST"))

  def test_configured_in_range_axes_are_homed(self):
    homed, _reason = self.homed_state(FakeAxesPort())
    self.assertTrue(homed)

  def test_power_up_ratio_means_homing_is_needed(self):
    port = FakeAxesPort()
    port.ratios[3] = 1000  # Z powered up since the last setup
    homed, reason = self.homed_state(port)
    self.assertFalse(homed)
    self.assertIn("Z motor", reason)

  def test_homing_settled_just_below_min_position_is_still_homed(self):
    port = FakeAxesPort()
    port.positions[2] = 78  # Y's MinPosition is 79 ticks; homing settled one below
    homed, _reason = self.homed_state(port)
    self.assertTrue(homed)

  def test_out_of_range_position_means_homing_is_needed(self):
    port = FakeAxesPort()
    port.positions[1] = -500
    homed, reason = self.homed_state(port)
    self.assertFalse(homed)
    self.assertIn("outside its travel", reason)

  def test_cycle_skips_homing_only_when_homed(self):
    for homed in (True, False):
      calls: List[Dict[str, Any]] = []

      async def run_demo(celigo: Any, plate_wait: Any, home: bool = True) -> None:
        calls.append({"home": home})

      async def no_op(_port: str) -> None:
        return None

      async def state(_port: str, homed: bool = homed) -> Any:
        return homed, "reason"

      with (
        self.subTest(homed=homed),
        patch.object(door_cycle, "ensure_stage_inside", no_op),
        patch.object(door_cycle, "homed_state", state),
        patch.object(door_cycle, "make_demo_celigo", lambda port: object()),
        patch.object(door_cycle, "run_demo", run_demo),
      ):
        asyncio.run(door_cycle.cycle("COM_TEST", 0.0))
        self.assertEqual(calls, [{"home": not homed}])


if __name__ == "__main__":
  unittest.main()
