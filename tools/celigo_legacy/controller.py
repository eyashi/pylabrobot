"""Build a controller-only ``Celigo`` for evaluating an instrument without vendor files.

``Celigo.setup()`` initializes motors, galvos, the camera, and homes axes, all from the
vendor configuration. This module instead builds a ``Celigo`` whose configuration has no
axes, galvos, IO, or channels, swaps in :class:`CeligoSerialIO` and a camera that refuses
to open, and performs only the transport and handshake portion of ``setup()``.

Nothing here moves a motor or changes an output. The calls it makes are the board's
ABORT, CONTROLLER_STATUS, SEND_CONFIG, SEND_MOTOR_CONFIG, and READ_DIG_PORT commands.
"""

import asyncio
import contextlib
from typing import Optional, cast
from unittest.mock import patch

from serial_io import CeligoSerialIO

from pylabrobot.io.ftdi import FTDI
from pylabrobot.revvity.celigo import (
  CalibrationConfig,
  Celigo,
  CeligoConfig,
  CeligoError,
  CeligoHardwareConfig,
  ControllerStatus,
  GalvoAxisOpticalCalibration,
  GalvoOpticalCalibration,
  HardwareDefaultConfig,
  NavigationConfig,
)


class NoCamera:
  """Camera placeholder: the camera is not connected during motor evaluation."""

  is_open = False
  width = 0
  height = 0

  async def setup(self) -> None:
    raise CeligoError("No camera is connected during legacy motor evaluation")

  async def stop(self) -> None:
    pass


def make_controller_only_config() -> CeligoConfig:
  """A ``CeligoConfig`` with no hardware components and zeroed calibration placeholders."""
  galvo_axis = GalvoAxisOpticalCalibration(
    magnifications={},
    logical_filter_offsets={},
    laser_center_voltage=0.0,
    uv_laser_center_voltage=0.0,
  )
  return CeligoConfig(
    hardware=CeligoHardwareConfig(),
    channel_descriptors=[],
    channels_by_magnification={3: {}},
    calibration=CalibrationConfig(
      microns_per_pixel_x=1.0,
      microns_per_pixel_y=1.0,
      image_width_pixels=0,
      image_height_pixels=0,
      image_to_stage_theta_radians=0.0,
      galvo_to_stage_theta_radians=0.0,
      calibrated_plate_corner_x=0.0,
      calibrated_plate_corner_y=0.0,
      calibrated_plate_to_stage_theta_radians=0.0,
      stage_x_scale=1.0,
      stage_y_scale=1.0,
      stage_shear=0.0,
      stage_x_shear_offset=0.0,
      stage_y_shear_offset=0.0,
      calibrated_z_position=0.0,
      calibrated_z_glass_plate_delta=0.0,
      z_plane_x_coeff=0.0,
      z_plane_y_coeff=0.0,
    ),
    hardware_defaults=HardwareDefaultConfig(
      default_calibrated_z=0.0,
      default_plate_x_corner_stage_coordinate=0.0,
      default_plate_y_corner_stage_coordinate=0.0,
      default_x_field_of_view_mm=0.0,
      default_y_field_of_view_mm=0.0,
      default_x_galvo_mm_per_volt=0.0,
      default_y_galvo_mm_per_volt=0.0,
    ),
    galvo_calibrations={},
    galvo_optical_calibration=GalvoOpticalCalibration(x=galvo_axis, y=galvo_axis),
    navigation=NavigationConfig(
      frame_overlap_x_mm=0.0,
      frame_overlap_y_mm=0.0,
      max_galvo_deflection_x_mm=0.0,
      max_galvo_deflection_y_mm=0.0,
    ),
  )


def make_celigo(
  port: str,
  config: Optional[CeligoConfig] = None,
  reply_timeout: float = 2.0,
) -> Celigo:
  """Construct a ``Celigo`` on ``port`` with the serial adapter and no camera."""
  # FTDI() and LumeneraCamera() are replaced before construction so neither libftdi nor
  # the Lumenera SDK is touched.
  with (
    patch("pylabrobot.revvity.celigo.celigo.FTDI"),
    patch("pylabrobot.revvity.celigo.celigo.LumeneraCamera", return_value=NoCamera()),
  ):
    celigo = Celigo(
      config=make_controller_only_config() if config is None else config,
      reply_timeout=reply_timeout,
    )
  celigo.io = cast(FTDI, CeligoSerialIO(port, baudrate=celigo.baudrate))
  return celigo


async def connect(celigo: Celigo) -> ControllerStatus:
  """Open the port and run the handshake portion of ``Celigo.setup()``.

  Mirrors ``setup()`` up to and including controller identification, then stops:
  no motor initialization, galvo calibration, camera, or homing.
  """
  io = celigo.io
  await io.setup()
  try:
    await io.set_baudrate(celigo.baudrate)
    await io.set_line_property(8, 0, 0)  # 8 data bits, 1 stop bit, no parity
    await io.set_latency_timer(celigo.latency_ms)
    await io.usb_purge_rx_buffer()
    await io.usb_purge_tx_buffer()
    for _ in range(2):
      with contextlib.suppress(CeligoError):
        await celigo.abort_controller_operation()
    status: Optional[ControllerStatus] = None
    last_error: Optional[CeligoError] = None
    for _ in range(3):
      try:
        status = await celigo.request_controller_status()
        break
      except CeligoError as exc:
        last_error = exc
        await asyncio.sleep(0.1)
    if status is None:
      raise CeligoError("Celigo did not return a valid controller status") from last_error
    celigo.controller_info = await celigo.request_controller_info()
  except BaseException:
    with contextlib.suppress(Exception):
      await io.stop()
    raise
  return status


async def disconnect(celigo: Celigo) -> None:
  """Close the port without sending further commands."""
  await celigo.io.stop()
