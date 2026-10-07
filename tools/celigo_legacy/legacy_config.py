"""Complete ``CeligoConfig`` for the legacy Celigo.

The hardware section (motors, galvos, I/O) is the measured vendor-format file
``config/USBIOHardwareConfig.config``, loaded with the driver's own
``CeligoHardwareConfig.from_xml``. The remaining sections normally come from the vendor's
optical and plate calibration files, which need a camera; until one is connected they are
the placeholders below, chosen so that:

* coordinate transforms are identity-scaled, with the sample origin placed at the X/Y
  minimum positions, just off the home sensors (rear, at the door end of X).
  ``Celigo.close_drawer_to_sample_mm(0, 0)`` therefore brings Y in first and parks the
  plate there. This is not the plate's real position under the optics.
* galvo centering moves the mirrors to 0 V, their measured power-on position.
* the only illumination channel is brightfield, at the vendor's typical 70 % intensity.
"""

from pathlib import Path

from pylabrobot.revvity.celigo import (
  CalibrationConfig,
  CeligoConfig,
  CeligoHardwareConfig,
  GalvoAxisOpticalCalibration,
  GalvoMagnificationCalibration,
  GalvoOpticalCalibration,
  HardwareDefaultConfig,
  IlluminationChannelConfig,
  NavigationConfig,
)

HARDWARE_CONFIG_PATH = Path(__file__).resolve().parent / "config" / "USBIOHardwareConfig.config"
MAGNIFICATION = 3  # no magnification changer is fitted; the driver's default is used
BRIGHTFIELD_INTENSITY_PERCENT = 70.0


def _park_pose(hardware: CeligoHardwareConfig) -> tuple[float, float]:
  """Stage X/Y (mm) at each axis's minimum position, next to its home sensor."""
  assert hardware.x_axis is not None and hardware.y_axis is not None
  return hardware.x_axis.min_position, hardware.y_axis.min_position


def _placeholder_calibration() -> CalibrationConfig:
  """Identity image and stage transforms; replace with the camera-derived calibration."""
  return CalibrationConfig(
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
  )


def _placeholder_galvo_optics() -> GalvoOpticalCalibration:
  """Center both galvos at 0 V for the only (default) magnification."""
  axis = GalvoAxisOpticalCalibration(
    magnifications={
      MAGNIFICATION: GalvoMagnificationCalibration(center_voltage=0.0, frame_size_volts=1.0)
    },
    logical_filter_offsets={},
    laser_center_voltage=0.0,
    uv_laser_center_voltage=0.0,
  )
  return GalvoOpticalCalibration(x=axis, y=axis)


def _brightfield_channel() -> IlluminationChannelConfig:
  return IlluminationChannelConfig(
    name="brightfield",
    display_name="Brightfield",
    logical_filter=1,
    bit_value=None,
    intensity_percent=BRIGHTFIELD_INTENSITY_PERCENT,
    lighting_io_name="eBrightFieldIntensity",
    strobe=False,
    z_offset_to_brightfield_mm=0.0,
    mm_per_pixel_x_correction_to_brightfield=1.0,
    mm_per_pixel_y_correction_to_brightfield=1.0,
  )


def load_legacy_config(hardware_path: Path = HARDWARE_CONFIG_PATH) -> CeligoConfig:
  """Measured hardware configuration plus placeholder calibration (see module docstring)."""
  hardware = CeligoHardwareConfig.from_xml(str(hardware_path))
  park_x, park_y = _park_pose(hardware)
  return CeligoConfig(
    hardware=hardware,
    channel_descriptors=[],
    channels_by_magnification={MAGNIFICATION: {"brightfield": _brightfield_channel()}},
    calibration=_placeholder_calibration(),
    hardware_defaults=HardwareDefaultConfig(
      default_calibrated_z=0.0,
      default_plate_x_corner_stage_coordinate=park_x,
      default_plate_y_corner_stage_coordinate=park_y,
      default_x_field_of_view_mm=0.0,
      default_y_field_of_view_mm=0.0,
      default_x_galvo_mm_per_volt=0.0,
      default_y_galvo_mm_per_volt=0.0,
    ),
    galvo_calibrations={},
    galvo_optical_calibration=_placeholder_galvo_optics(),
    navigation=NavigationConfig(
      frame_overlap_x_mm=0.0,
      frame_overlap_y_mm=0.0,
      max_galvo_deflection_x_mm=0.0,
      max_galvo_deflection_y_mm=0.0,
    ),
    magnification=MAGNIFICATION,
  )
