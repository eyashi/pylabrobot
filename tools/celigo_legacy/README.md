# Legacy Celigo: handoff notes

Context for anyone (human or agent) integrating this instrument into a lab scheduler, or
continuing the imaging work. Written 2026-10-08 at the end of the motion-control
evaluation, on branch `celigo-legacy-motor-eval` of the PyLabRobot repository.

## TL;DR

- An **old Revvity/Nexcelom/Cyntellect Celigo** is driven by the **PyLabRobot `Celigo`
  driver** (`pylabrobot/revvity/celigo/`), with no vendor software and no vendor config files.
- **Motion works end to end**: X/Y stage, Z focus, and the 4-position filter wheel are
  mapped, home with the driver, and the stage presents and retrieves a plate through the
  door (`door_demo.py`, `door_cycle.py`).
- **Galvos** calibrate and move. **Brightfield** lights. **Fluorescence, laser, and camera
  are not done**: the camera interface board had not arrived, and this unit's camera
  differs from the Lumenera the driver supports.
- The configuration is a vendor-format `config/USBIOHardwareConfig.config` built from
  measurements, plus `legacy_config.py`, which supplies **placeholder** optical/plate
  calibration until a camera can measure it.

## Hard rules

1. **Never drive the hardware without explicit, per-run approval from the operator**
   (repository `AGENTS.md`). Read-only queries were also run only with approval.
2. **Y leaves through the door only when X is at its home (right) end.** The door is in
   line with X's home position. Any X move while Y is out can hit the door frame.
3. **Z positive is UP, toward the sample**, which is the collision direction. Homing and
   `open_drawer()` move Z down. Z's upper travel is capped at 5 mm (provisional).
4. **One process owns COM4.** The driver is asyncio-based and serializes commands with a
   lock inside one event loop. Do not open the port from two processes or loops.
5. Before changing `pylabrobot/` itself, read `docs/contributor_guide/contributing.md`. Its
   rules include: docstrings everywhere, define-before-use, no `getattr`/`setattr`-style
   reflection, and a failing test before every bug fix.

## The hardware

| Item | Value |
|---|---|
| Controller | FTDI `0403:6001`, Windows VCP driver, **COM4**, 230400 8N1, latency timer **2 ms** (set in Device Manager; replug the USB after changing it) |
| Controller firmware | 1.3.0 (length-prefixed motor framing, opcode 47) |
| Motors | 4 × AllMotion EZStepper V7.21, addresses 1-4, 256 microsteps/step, 200 steps/rev |
| Motor 1, **X** | Side to side. Positive = stage moves **left**. Home sensor (bit 2) at the **right** end, where the door is; far sensor (bit 3) at the left. 200 ticks/turn. ~129 mm of travel. |
| Motor 2, **Y** | Front/back. Positive = toward the **front**, out of the door. Home sensor (bit 2) at the rear; far sensor (bit 3) at the front, 253.0 mm out. 200 ticks/turn. |
| Motor 3, **Z** | Focus. Positive = objective **up**. Home sensor (bit 2) at the bottom. 1600 ticks/turn. Very fine drive. |
| Motor 4, wheel | Dichroic filter wheel: 4 positions, 8000 ticks/rev (4 motor turns), 2000 ticks per position, **has an encoder index**. Position-one flag *clears* bit 2 (`LimitPolarity` 1), ~663 ticks wide. |
| Sensors | On X/Y/Z, raw bit set = triggered (`LimitPolarity` 0). The X/Y/Z encoders have **no index**. |
| Lead screws | Assumed 0.1 inch (0.0127 mm/tick). A hand measurement gave ~2.65 mm/turn, so calibrate against a plate. |
| Galvos | Present. Calibration sweep passes; +/-0.5 V steps settle in ~30 ms; rest position 0 V. |
| Brightfield | Analog output 0, 0-10 V mapping; visible above the stage at >= 25 %. |
| I/O | All digital and analog inputs read 0. The door, lid, and interlock change **no** signal software can see. Status flag 4 ("interlock open") is always set; treat it as meaningless here. |

Every run's report and byte log is in `runs/` (gitignored, local to the evaluation PC).

## Driver changes made (in `pylabrobot/revvity/celigo/`)

All are general, tested, and suitable to upstream:

1. **`Celigo(..., camera_enabled=False)`**: setup and stop skip the camera; acquisition
   methods raise `CeligoError("This Celigo was constructed with camera_enabled=False")`.
2. **`Celigo.setup(home=False)`**: everything setup normally does except homing. Instead,
   it adopts each linear axis's current encoder position (`LinearAxis.assume_homed`). Only
   valid when the axes were homed since the motor controllers last powered up. The filter
   wheel stays unreferenced until homed.
3. **`LinearAxis.assume_homed()`** accepts positions within the axis's fine-position
   window of its bounds. Homing's final move to `MinPosition` may settle one tick outside
   the exact bound, and was previously rejected.

Homing needed **no** driver change. The existing `HomeType` `NormalWithHardstopCheck`
uses the firmware's index-free home (`N1` + `Z`), which on this hardware stops at the
home-sensor edge and zeroes the counters, repeatable to 1 encoder tick.

## Configuration

- `config/USBIOHardwareConfig.config`: vendor format, read by
  `CeligoHardwareConfig.from_xml`. Each block is commented MEASURED or PROVISIONAL.
  - X/Y: 30 mm/s, 45 mm/s^2, currents 65/20/55 % (vendor sample values; the same lead
    screw as the vendor's sample config). Homing deliberately slow (1.0 / 0.5 mm/s) for
    edge precision. `HomingShortMove` 500 ticks, larger than the measured sensor depth.
  - Z: 0.32 mm/s, travel 0.5-5 mm (provisional), 0.001 mm/tick (**assumed**).
  - Wheel: 400 ticks/s, `HomeOffset` 58 (centers position 1 on its flag).
  - Galvos: +/-1 V, not inverted (**placeholder** range).
  - I/O: brightfield lighting on analog 0, `FLOnOff` strobe on digital bit 6.
- `legacy_config.load_legacy_config()` returns the full `CeligoConfig`: that hardware file
  plus **placeholder** calibration. The transforms are identity-scaled with the sample
  origin at stage (1, 1) mm (both axes just off home), so
  `close_drawer_to_sample_mm(0, 0)` parks the plate there. Galvo centers are 0 V, and
  brightfield is at 70 %. None of this is the real optical calibration.

## Using the driver from another program

Everything is `async`. A minimal pattern (Python, run from `tools/celigo_legacy` or with
it on `sys.path`):

```python
from door_demo import make_demo_celigo        # Celigo(legacy config, no camera, COM adapter)
from door_cycle import homed_state            # query-only "still homed?" check
from door_demo import ensure_stage_inside     # refuses if Y is out of the door

celigo = make_demo_celigo("COM4")             # move_timeout=180 s for long moves
await ensure_stage_inside("COM4")             # BEFORE setup: setup homes X before Y
homed, reason = await homed_state("COM4")
await celigo.setup(home=not homed)            # ~1-2 min with homing, seconds without
try:
  await celigo.open_drawer()                  # Z down, Y in, X to the door, Y out to front sensor
  # ... arm places or removes the plate here ...
  await celigo.close_drawer_to_sample_mm(0, 0)  # Y in first, then park at (1, 1) mm
finally:
  await celigo.stop()                         # outputs off, port closed; always call it
```

Useful driver calls:

- `await celigo.x_axis.request_position()` (likewise `y_axis`, `z_axis`): position in mm.
- `await celigo.y_axis.request_is_positive_limit_active()`: True when the stage is out at
  the door. That is the "door open" state; there is no separate door sensor.
- `await celigo.set_brightfield_enabled(True/False)` and `await celigo.turn_off_illumination()`.
- `await celigo.run_self_test()`: read-only diagnostics (status, identity, motor map,
  encoders, ratios). With `run_active_checks=True` it also centers the galvos and tries a
  camera frame, which fails, as expected, when `camera_enabled=False`.
- Errors are `CeligoError` (controller, motor, or validation) and `TimeoutError` (a move
  did not finish). After either, call `stop()` and re-run `setup()`.

Why the COM-port adapter (`serial_io.CeligoSerialIO`): the driver's own transport is
pylibftdi, which on Windows needs libftdi/libusb DLLs and a Zadig WinUSB driver in place
of FTDI's. The adapter implements the same calls over the existing VCP driver. If the
integration host is Linux, the driver's native FTDI transport is the intended path
(`Celigo(usb_address=...)`); verify it there.

## Integrating with the arm and scheduler

Recommendations; none of this is built yet:

- **Wrap the Celigo in one long-lived service process** that owns COM4 and the event
  loop, exposing coarse commands such as `initialize`, `open_door`, `close_door`,
  `status`, and `shutdown`. If the scheduler is not Python, put an HTTP/gRPC/CLI boundary
  on that service rather than starting a Python process per action (each start costs a
  setup).
- **Split open and close into separate commands** so the scheduler sequences:
  `open_door` → arm places the plate → `close_door`. `door_demo.py` and `door_cycle.py`
  combine them around a keyboard or timed wait; that is for demos only.
- **Hand-off pose for the arm**: after `open_drawer()`, Y is on its front sensor
  (~253 mm), X is on its home sensor (right end), and Z is at 0.5 mm. Teach the arm
  against that physical pose; it is repeatable because it ends on sensors.
- **Interlock the arm in software**: the arm must not enter while any Celigo axis moves.
  The driver's calls return only after the motion settles, so "call returned" means
  stopped.
- **Startup state**: after a power cycle the motors lose their positions and need homing
  (`homed_state` detects this). If the stage was left out of the door, bring it in before
  `setup()`. A future fix is to home Y before X on this unit.
- Typical durations were not measured precisely. Moves are seconds at 30 mm/s; homing
  from near home is ~1-2 min; homing Y from the door is ~4-5 min at the slow homing speed.

## Tools in this directory

| Script | Purpose | Moves hardware? |
|---|---|---|
| `serial_io.py`, `controller.py` | COM-port adapter; controller-only `Celigo` for probing | — |
| `probe_controller.py`, `probe_motors.py`, `io_survey.py` | Read-only identity, motor, and I/O surveys | No |
| `wiggle_motor.py`, `jog_motor.py` | Raw microstep moves on **unconfigured** motors only (they refuse after driver setup) | Yes |
| `home_stepwise.py` | Driver homing one stage at a time under a watchdog (`--axis x/y/z`, `--through`, `--driver-home`) | Yes |
| `wheel_index_test.py` | Wheel index check; `--map` derives `HomeOffset` and runs `FilterWheel.home()` | Yes |
| `galvo_test.py` | Galvo calibration and small steps | Galvos |
| `illumination_test.py`, `brightfield_test.py` | Analog-output and driver-path brightfield tests | Light only |
| `door_demo.py` | Full flow: home, open, wait, close | Yes |
| `door_cycle.py` | Same, but skips homing when the axes are still homed | Yes |
| `camera_probe.py` | Allied Vision camera bring-up (`info`, `capture`, `roi`); see `CAMERA_HANDOFF.md` | Camera only |
| `legacy_config.py`, `config/` | The instrument configuration | — |

Tests: `.venv\Scripts\python -m pytest tools/celigo_legacy pylabrobot/revvity/celigo`
(about 250 tests, all offline, using simulated controllers). Lint and type checks:
`ruff check`, `ruff format --check`, and `mypy` on the touched paths. Don't run the whole
repository suite; it is large and not needed.

## Future work: imaging

Camera bring-up on the FireWire PC: follow [`CAMERA_HANDOFF.md`](CAMERA_HANDOFF.md).

The camera interface board was pending, and this unit's camera **is not** the Lumenera
the driver supports. In order:

1. **Camera driver.** The driver consumes the `CeligoCamera` protocol
   (`pylabrobot/revvity/celigo/camera.py`): `exposure_ms`, `gain`, `width`, `height`,
   `is_open`, `setup()`, `stop()`, `set_exposure()`, `set_gain()`,
   `set_frame_format(width, height, x_offset, y_offset)`, and `capture(flush_frames)`, which
   returns a `CameraFrame` (raw monochrome bytes, 8 or 16 bit). Implement that for the new
   camera. `Celigo` constructs `LumeneraCamera` internally, so either assign
   `celigo.camera = NewCamera()` after construction (with `camera_enabled=True`), or add a
   constructor parameter that accepts a camera (cleaner; upstreamable).
2. **Image geometry.** Setup applies `CalibrationConfig.image_width_pixels` and
   `image_height_pixels` as the camera ROI and rejects mismatched frames. Set them for the
   new sensor.
3. **Calibrations the placeholders stand in for** (`legacy_config.py`), each measurable
   once images exist:
   - microns per pixel and image-to-stage rotation (`CalibrationConfig`);
   - plate-corner stage coordinates and stage scale/shear: register a plate. The real park
     and imaging positions follow from these;
   - Z mm per tick and the in-focus Z for a plate (`calibrated_z_position`), then autofocus;
   - galvo voltage range, inversion, imaging-center voltages, frame size, and the
     volts-to-mm transforms (`GalvoOpticalCalibration`, `galvo_calibrations`);
   - filter-wheel position centers in the light path (positions are currently flag-based);
   - the X/Y lead screw (verify 0.0127 mm/tick against the 9 mm well pitch).
4. **Camera trigger lines.** Controller signal diagnostics read camera busy = 1 and
   integration = 2 (driver: "unavailable") with no camera attached. Revisit with the camera.
5. **Fluorescence.** Untested. PyLabRobot's fixture map puts intensity on analog 2,
   filter select `FLBit0`/`FLBit1` on digital bits 4/5, strobe on bit 6, and lamp power
   (`ExcitationLampPower`) on bit 7. Find out the lamp type first. Arc lamps need warm-up
   and minimum power-toggle intervals, which the driver enforces.
6. **Laser.** Never touched; the driver keeps it disabled unless `allow_laser=True`.
   Needs an explicit laser-safety review before any use.

## Known driver findings worth reporting upstream

- `home_imaging_axes()` homes X before Y. On this unit that can drive an extended stage
  into the door frame.
- The firmware's index search (`Z` with `N2`/`N6`) does not honor its distance argument.
  On an axis without an index it runs until something else stops it.
- `LinearAxis.home()` backs off only `HomingShortMove` from the sensor; an axis starting
  deeper inside its sensor than that cannot home. This config sizes the value to fit.
- A command cancelled mid-flight leaves its reply in flight, and the next command then
  reads a stale reply (caught cleanly by the sequence check, as a `CeligoError`).
