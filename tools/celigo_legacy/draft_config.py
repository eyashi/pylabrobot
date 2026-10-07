"""Hand-written axis configuration for the legacy Celigo, from measured motor behavior.

Measured (see tools/celigo_legacy/runs/): 51200 microsteps per motor turn, 200 encoder ticks
per turn on the stage motors (256 microsteps per tick), home sensor on input bit 2 (raw bit
set = triggered) at the negative end of each stage axis. Lead screws are assumed to be
0.1 inch (0.0127 mm per tick) until calibrated against a plate.

Speeds are deliberately slow: 0.508 mm/s = 40 ticks/s = 0.2 motor turns/s, the rate
already exercised by ``jog_motor.py``. Values not yet measured (response time, error
windows) follow the trimmed vendor sample in ``pylabrobot/revvity/celigo/tests``.
"""

from pylabrobot.revvity.celigo import LinearAxisConfig

STAGE_MM_PER_ENCODER_TICK = 0.0127
STAGE_MICROSTEPS_PER_ENCODER_TICK = 256.0


def stage_axis_config(motion_name: str, axis_index: int, home_type: str) -> LinearAxisConfig:
  """A conservative stage-axis configuration homing toward its negative-end sensor."""
  return LinearAxisConfig(
    motion_name=motion_name,
    config_version=1001,
    motor_type=0,
    comm_index=1,
    controller_index=0,
    axis_index=axis_index,
    enabled=True,
    max_velocity=0.508,
    max_acceleration=5.0,
    max_deceleration=5.0,
    max_s_acceleration=0,
    moderate_acceleration=5.0,
    minimum_acceleration=5.0,
    moderate_s_acceleration=0,
    minimum_s_acceleration=0,
    s_curve_support=False,
    home_type=home_type,
    homing_velocity=0.508,
    index_velocity=0.254,
    # Ticks moved off the sensor before the index search: one motor turn (~2.5 mm), well
    # beyond the ~14-tick reversal slack measured on Y.
    homing_short_move=200,
    home_offset=0.0,
    positive_limit=False,  # far-end sensors not yet mapped
    negative_limit=True,
    limit_polarity=0,  # raw bit set = triggered
    invert_axis_direction=False,
    default_positive_direction=True,  # keep the motor's native direction (F0)
    moving_current_percentage=30,
    holding_current_percentage=10,
    loading_current_percentage=30,
    moving_overload_limit=500,
    mode_enable_limits=True,
    mode_enable_step_and_direction=False,
    mode_enable_position_correction=False,  # open loop until the basics are proven
    mode_enable_motor_slave_to_encoder=False,
    coarse_position_error_window=100,
    fine_position_error_window=10,
    gain=1,
    encoder_to_motor_tick_ratio=STAGE_MICROSTEPS_PER_ENCODER_TICK,
    backlash_compensation=0,
    motor_response_time=1,
    min_position=0.0,
    # Conservative until the far-end sensor is found: 20 mm from home.
    max_position=20.0,
    mm_per_encoder_tick=STAGE_MM_PER_ENCODER_TICK,
  )


def y_axis_config(home_type: str = "Normal_Accurate") -> LinearAxisConfig:
  """Motor 2: front/back stage. Positive moves toward the front; home is at the rear."""
  return stage_axis_config("Y Axis (legacy draft)", 2, home_type)
