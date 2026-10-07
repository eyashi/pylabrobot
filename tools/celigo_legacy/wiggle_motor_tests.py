"""Offline tests for the single-motor wiggle, using a simulated moving motor."""

import argparse
import asyncio
import re
import struct
import unittest
from typing import Any, Dict, List, Optional
from unittest.mock import patch

import serial_io
import wiggle_motor
from probe_motors_tests import FakeMotorPort, _oem_reply

_MOVE = re.compile(r"^m(\d+)V(\d+)([PD])(\d+)R$")


class FakeMovingPort(FakeMotorPort):
  """Motors that take three status polls to finish a move; encoder = microsteps / 64."""

  def __init__(
    self,
    flag_threshold: Optional[int] = None,
    error_on_move: bool = False,
    encoder_stall_at: Optional[int] = None,
  ) -> None:
    super().__init__()
    self.encoder_stall_at = encoder_stall_at
    self.positions = {address: 0 for address in self.encoders}
    self.targets = dict(self.positions)
    self.flag_threshold = flag_threshold
    self.error_on_move = error_on_move
    self.moves: List[str] = []
    self.terminated: List[int] = []

  def _flags(self, address: int) -> int:
    crossed = self.flag_threshold is not None and self.positions[address] >= self.flag_threshold
    return 7 if crossed else 3

  def _payload(self, opcode: int) -> bytes:
    if opcode != 47:
      return super()._payload(opcode)
    request = self._last_request
    address = int(chr(request[1]))
    tokens = request[3:-2].decode("ascii")
    self.motor_commands.append(tokens)
    status = 0x60
    move = _MOVE.match(tokens)
    if move:
      self.moves.append(tokens)
      sign = 1 if move.group(3) == "P" else -1
      self.targets[address] = self.positions[address] + sign * int(move.group(4))
      text = ""
    elif tokens == "T":
      self.terminated.append(address)
      self.targets[address] = self.positions[address]
      text = ""
    elif tokens == "Q":
      remaining = self.targets[address] - self.positions[address]
      if remaining:
        if self.error_on_move:
          status = 0x49  # ready bit clear, error code 9
        else:
          step = max(1, abs(remaining) // 2) if abs(remaining) > 1 else 1
          self.positions[address] += step if remaining > 0 else -step
          status = 0x40  # busy
      text = ""
    elif tokens == "?0":
      text = str(self.positions[address])
    elif tokens == "?8":
      position = self.positions[address]
      if self.encoder_stall_at is not None:
        position = min(position, self.encoder_stall_at)  # the mechanism stopped moving
      text = str(position // 64)
    elif tokens == "?4":
      text = str(self._flags(address))
    else:
      return super()._payload(opcode)
    reply = _oem_reply(text, status)
    return struct.pack(">HH", 0, len(reply)) + reply


def _args(**changes: Any) -> argparse.Namespace:
  values: Dict[str, Any] = {
    "port": "COM_TEST",
    "address": 2,
    "direction": "positive",
    "distance": 2560,
    "current": 30,
    "velocity": 2560,
    "pause": 0.0,
    "continue_past_flags": False,
    "reply_timeout": 0.5,
  }
  values.update(changes)
  return argparse.Namespace(**values)


class WiggleTests(unittest.TestCase):
  def run_wiggle(self, port: FakeMovingPort, **changes: Any) -> Dict[str, Any]:
    report: Dict[str, Any] = {}
    with (
      patch.object(serial_io.serial, "serial_for_url", return_value=port),
      patch.object(wiggle_motor, "POLL_INTERVAL", 0.0),
    ):
      asyncio.run(wiggle_motor.run(_args(**changes), report))
    return report

  def test_moves_out_and_back_by_the_same_distance(self):
    port = FakeMovingPort()
    report = self.run_wiggle(port)
    self.assertNotIn("error", report)
    self.assertEqual(port.moves, ["m30V2560P2560R", "m30V2560D2560R"])
    self.assertEqual(port.positions[2], 0)
    self.assertEqual(report["summary"]["microsteps_taken"], 2560)
    self.assertEqual(report["summary"]["encoder_ticks_per_microstep"], 40 / 2560)
    self.assertTrue(report["summary"]["returned_microsteps"])

  def test_only_the_requested_motor_moves(self):
    port = FakeMovingPort()
    self.run_wiggle(port, address=3)
    self.assertEqual(port.positions, {1: 0, 2: 0, 3: 0, 4: 0})
    for tokens in port.motor_commands:
      self.assertTrue(
        tokens in ("&", "Q", "T") or tokens.startswith("?") or _MOVE.match(tokens), tokens
      )

  def test_negative_direction_moves_negative_first(self):
    port = FakeMovingPort()
    self.run_wiggle(port, direction="negative", distance=1000)
    self.assertEqual(port.moves, ["m30V2560D1000R", "m30V2560P1000R"])

  def test_flag_change_stops_and_returns_the_steps_taken(self):
    port = FakeMovingPort(flag_threshold=1000)
    report = self.run_wiggle(port)
    self.assertTrue(report["outward_stopped_on_flag_change"])
    self.assertIn(2, port.terminated)
    taken = report["summary"]["microsteps_taken"]
    self.assertLess(taken, 2560)
    self.assertEqual(port.moves[1], f"m30V2560D{taken}R")
    self.assertEqual(port.positions[2], 0)
    self.assertEqual([(c["from"], c["to"]) for c in report["flag_changes"]], [(3, 7), (7, 3)])

  def test_continue_past_flags_logs_without_stopping(self):
    port = FakeMovingPort(flag_threshold=1000)
    report = self.run_wiggle(port, continue_past_flags=True)
    self.assertFalse(report["outward_stopped_on_flag_change"])
    self.assertEqual(port.terminated, [])
    self.assertEqual(port.moves, ["m30V2560P2560R", "m30V2560D2560R"])
    self.assertEqual([(c["from"], c["to"]) for c in report["flag_changes"]], [(3, 7), (7, 3)])

  def test_motor_error_terminates_the_motor(self):
    port = FakeMovingPort(error_on_move=True)
    report = self.run_wiggle(port)
    self.assertIn("error 9", report["error"])
    self.assertEqual(port.terminated, [2])

  def test_refuses_a_driver_configured_motor(self):
    port = FakeMovingPort()
    port.encoder_ratio = 256000
    report = self.run_wiggle(port)
    self.assertIn("Power-cycle", report["error"])
    self.assertEqual(port.moves, [])

  def test_undetected_address_is_refused(self):
    port = FakeMovingPort()
    report = self.run_wiggle(port, address=4)  # the fake controller reports motors 1-3
    self.assertIn("not detected", report["error"])
    self.assertEqual(port.moves, [])

  def test_arguments_are_capped(self):
    for flag, value in (("--distance", "51201"), ("--current", "51"), ("--velocity", "0")):
      with self.subTest(flag=flag), self.assertRaises(SystemExit):
        wiggle_motor.main(["--port", "COM_TEST", "--address", "2", flag, value])


if __name__ == "__main__":
  unittest.main()
