"""Offline tests for the read-only motor probe, using a simulated controller port."""

import argparse
import asyncio
import struct
import unittest
from typing import Any, Dict, List
from unittest.mock import patch

import probe_motors
import serial_io
from probe_controller_tests import FakeControllerPort

_STX = 0x02
_ETX = 0x03


def _oem_reply(text: str, status: int = 0x60) -> bytes:
  frame = bytes([_STX]) + b"0" + bytes([status]) + text.encode("ascii") + bytes([_ETX])
  checksum = 0
  for value in frame:
    checksum ^= value
  return frame + bytes([checksum])


class FakeMotorPort(FakeControllerPort):
  """Adds EZStepper replies tunneled through MOTOR_CMD_QUERY_WLEN (opcode 47)."""

  def __init__(self) -> None:
    super().__init__(firmware=0x010300)
    self.encoders = {1: 100, 2: 200, 3: 300, 4: 400}
    # Successive encoder readings per address; the last value repeats.
    self.encoder_sequences: Dict[int, List[int]] = {}
    self.motor_commands: List[str] = []
    self.encoder_ratio = 1000  # power-up value

  def _payload(self, opcode: int) -> bytes:
    if opcode != 47:
      return super()._payload(opcode)
    request = self._last_request
    assert request[0] == _STX and request[-2] == _ETX
    address = int(chr(request[1]))
    assert chr(request[2]) == "1"
    tokens = request[3:-2].decode("ascii")
    self.motor_commands.append(tokens)
    if tokens == "&":
      text = "EZHR17EN AllMotion V7.16"
    elif tokens == "?8":
      sequence = self.encoder_sequences.get(address)
      if sequence:
        self.encoders[address] = sequence.pop(0) if len(sequence) > 1 else sequence[0]
      text = str(self.encoders[address])
    elif tokens == "?aE":
      text = str(self.encoder_ratio)
    elif tokens == "?4":
      text = "4"
    else:
      text = ""
    reply = _oem_reply(text)
    return struct.pack(">HH", 0, len(reply)) + reply

  def write(self, data: bytes) -> int:
    self._last_request = data[11:]
    return super().write(data)


def _args(**changes: Any) -> argparse.Namespace:
  values: Dict[str, Any] = {
    "port": "COM_TEST",
    "mode": "snapshot",
    "addresses": None,
    "seconds": 0.3,
    "interval": 0.05,
    "reply_timeout": 0.5,
  }
  values.update(changes)
  return argparse.Namespace(**values)


class ProbeMotorsTests(unittest.TestCase):
  def run_probe(self, port: FakeMotorPort, args: argparse.Namespace) -> Dict[str, Any]:
    report: Dict[str, Any] = {}
    with patch.object(serial_io.serial, "serial_for_url", return_value=port):
      asyncio.run(probe_motors.run(args, report))
    return report

  def test_snapshot_reads_every_detected_motor(self):
    port = FakeMotorPort()
    report = self.run_probe(port, _args())
    self.assertEqual(report["addresses"], [1, 2, 3])
    motor = report["motors"]["2"]
    self.assertEqual(motor["firmware"], (7, 16))
    self.assertEqual(motor["encoder_ticks"], 200)
    self.assertEqual(motor["encoder_ratio"], 1.0)
    self.assertEqual(motor["input_flags"]["text"], "4")
    self.assertEqual(set(motor["raw_queries"]), set(probe_motors.RAW_QUERIES))

  def test_sends_only_queries(self):
    port = FakeMotorPort()
    self.run_probe(port, _args(addresses=[1, 4]))
    self.run_probe(port, _args(mode="watch", addresses=[1]))
    self.assertTrue(port.motor_commands)
    for tokens in port.motor_commands:
      self.assertTrue(tokens in ("&", "Q") or tokens.startswith("?"), tokens)
      self.assertFalse(tokens.endswith("R"), tokens)

  def test_watch_reports_encoder_changes(self):
    port = FakeMotorPort()
    port.encoder_sequences[3] = [300, 300, 350]  # moved by hand between polls
    changes = self.run_probe(port, _args(mode="watch", addresses=[3]))["changes"]
    self.assertEqual(
      [(change["encoder_ticks"], change["delta_ticks"]) for change in changes],
      [(300, None), (350, 50)],
    )

  def test_rejects_non_query_tokens(self):
    with self.assertRaises(ValueError):
      asyncio.run(probe_motors._query(object(), "P100"))  # type: ignore[arg-type]


if __name__ == "__main__":
  unittest.main()
