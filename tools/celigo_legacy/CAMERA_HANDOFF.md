# Camera bring-up handoff (camera PC)

You are on the PC that holds the FireWire card connected to the legacy Celigo's camera.
Your job is to verify `AlliedVisionCamera` against the real camera with the steps below,
fix what fails, and report results in the format at the end. The motor controller (COM4)
is **not** on this PC, so nothing here can move the stage or switch on light by itself.

Read only this file to start. Open code only when a step's failure points into it.

## Rules

- Ask the operator before **every** run that talks to the camera (repository `AGENTS.md`:
  approval does not carry over between runs). Camera runs move nothing and light nothing.
- Never run the whole repository test suite. Use the scoped command in step 0.
- Read images by viewing the PNGs the tool writes (downsampled, 1024 px). Don't read the
  `features` array of a report in full; filter it (see step 1).
- Code style rules are in `docs/contributor_guide/contributing.md`. Read it before editing
  `pylabrobot/`.

## What exists (written and tested offline against a simulated SDK)

| Path | What it is |
|---|---|
| `pylabrobot/revvity/celigo/allied_vision_camera.py` | `AlliedVisionCamera`: `ctypes` binding to **Vimba 6** `VimbaC.dll`, implementing the `CeligoCamera` protocol (`setup`, `stop`, `set_exposure` (ms), `set_gain`, `set_frame_format`, `capture`). Also `list_features()`. |
| `pylabrobot/revvity/celigo/tests/allied_vision_camera_tests.py` | Unit tests with `_FakeVimbaLibrary`. |
| `pylabrobot/revvity/celigo/celigo.py` | `Celigo(..., camera=AlliedVisionCamera())` uses that camera instead of Lumenera. |
| `tools/celigo_legacy/camera_probe.py` | Bring-up stages `info`, `capture`, `roi`. Writes `runs/celigo_camera_<stage>_<time>.json` and PNGs, prints a summary, exits 0 on success. |

How the driver works, in brief (so you don't have to read it):

- `setup()`: `VmbStartup` → poll `VmbCamerasList` for up to 3 s → `VmbCameraOpen` (full
  access) → `AcquisitionMode=SingleFrame` (required) → `TriggerMode`, `ExposureAuto`, and
  `GainAuto` set to `Off` if present (results go in `setup_notes`) → `PixelFormat`
  (`Mono16` by default) → selects the first exposure feature of `ExposureTime` /
  `ExposureTimeAbs` (value assumed to be **µs**) and the first gain feature of `Gain` /
  `GainAbs` / `GainRaw` → reads `Width`/`Height`/`OffsetX`/`OffsetY` → announces one frame
  sized by `PayloadSize` → `VmbCaptureStart`.
- `capture(flush_frames)`: for each of `flush_frames + 1` frames, it queues the frame,
  runs `AcquisitionStart`, calls `VmbCaptureFrameWait` (timeout = exposure + 5 s), then
  `AcquisitionStop`. It checks frame status, size, and pixel format, and returns the
  last frame as `CameraFrame`.
- `set_frame_format(w, h)`: ends capture, zeroes the offsets, checks the size against
  the `Width`/`Height` maximum and increment, sets the size and centered offsets, reads
  everything back, and restarts capture.

## Assumptions to verify (each maps to a step)

| # | Assumption | Step |
|---|---|---|
| A1 | Vimba 6 and its 1394 transport layer and driver see the card and the camera | 1 |
| A2 | The camera is a Pike F-505B (datasheet: 2452 x 2054, 14-bit ADC, Mono8/Mono16, IEEE 1394b) | 1 |
| A3 | `AcquisitionMode` has `SingleFrame`; a software `AcquisitionStart` produces a frame with `TriggerMode=Off` | 1, 2 |
| A4 | The exposure feature is in **µs** (the driver divides by 1000) | 2 |
| A5 | Mono16 bytes are little-endian and the 14-bit data is MSB-aligned (max near 65535) | 3 |
| A6 | `ROI`: offsets and size behave as written in `set_frame_format` | 4 |

## Step 0: environment (once)

1. Install **Vimba 6.0 or 6.1** (not Vimba X, which has no FireWire support) from Allied
   Vision. In the installer, include the **IEEE 1394 transport layer**. Then run
   **Vimba Driver Installer** and assign the *Allied Vision 1394* driver to the FireWire
   card. Reboot if it asks.
2. Ask the operator to open **Vimba Viewer** and confirm the camera streams. If the viewer
   can't see it, the problem is the driver, not this code: stop and fix the
   installation first.
3. Python: a **64-bit** CPython 3.9+ (the DLL is 64-bit). Do not reuse a `.venv` copied
   from the other PC. Create a fresh one:
   ```
   python -m venv .venv
   .venv\Scripts\python -m pip install -e ".[serial]" pytest
   ```
4. Offline tests (no hardware, ~20 s):
   `.venv\Scripts\python -m pytest -q tools/celigo_legacy pylabrobot/revvity/celigo`.
   All tests must pass before you touch the camera.
5. The DLL is found automatically from `%VIMBA_HOME%` or
   `C:\Program Files\Allied Vision\Vimba_*\VimbaC\Bin\Win64\VimbaC.dll`. If it isn't, pass
   `--sdk-library <path>` to every `camera_probe.py` command.

## Step 1: identity and features (A1, A2, A3)

```
.venv\Scripts\python tools\celigo_legacy\camera_probe.py info
```

**Pass:** `"setup": "ok"`; `camera.model` names an F-505B; `geometry` is the full sensor
(expected 2452 x 2054, bit_depth 16); `setup_notes` contains `TriggerMode=Off`;
`exposure_feature` and `gain_feature` are set.

Pull only the features that matter (PowerShell):
```
$r = Get-Content (Get-ChildItem tools\celigo_legacy\runs\celigo_camera_info_*.json | Sort LastWriteTime | Select -Last 1) | ConvertFrom-Json
$r.features | ? { $_.name -match 'Exposure|Gain|Width|Height|Offset|PixelFormat|Trigger|Acquisition|PayloadSize|Bit' } | ft name,data_type,unit,value,error
```
For the exposure feature, confirm `unit` is `us` (A4).

| Failure | Meaning and fix |
|---|---|
| `Could not load the Vimba 6 C library` | Wrong path or a 32-bit Python. Pass `--sdk-library`; check `python -c "import struct;print(struct.calcsize('P')*8)"` prints 64. |
| `Vimba found no camera ... interfaces: none` or no `FireWire:` interface | 1394 transport layer or driver missing (step 0.1). |
| `FireWire:` interface present but no camera | Cable or power, or another program (Vimba Viewer) holds the camera. Close it. |
| `set AcquisitionMode=SingleFrame failed` | Read the `AcquisitionMode` value in the feature list. If only `Continuous` exists, change `_setup_sync` to use continuous mode, and change `_acquire_frame_sync` to stop acquisition after one frame and flush. Add a test to the fake. |
| `set PixelFormat=Mono16 failed` | List the `PixelFormat` value; rerun with `--pixel-format Mono8`; report. |
| `Camera has none of the features ExposureTime, ...` | Find the real exposure/gain names in the list and add them to `EXPOSURE_FEATURES` / `GAIN_FEATURES` (note their units). |
| `StructSize` error from `list features` | Compare `_VmbFeatureInfo` with `VimbaC.h` in the Vimba install (`VimbaC\Include`). |

## Step 2: dark capture, timing, and exposure units (A3, A4)

```
.venv\Scripts\python tools\celigo_legacy\camera_probe.py capture --exposures-ms 1 10 100 1000
```

**Pass:** 4 frames; `applied_exposure_ms` ≈ requested; `elapsed_s` for 1000 ms is
**≥ 1.0** (and < ~1.5); for 1 ms it's well under 1 s. This proves the exposure unit is µs.
View one PNG: it should be a dark, uniform frame with no stripes or shearing.

| Failure | Meaning and fix |
|---|---|
| `No frame within ... ms ... hardware trigger` | The camera still waits for a trigger. Check `setup_notes` and the `Trigger*` features. Set the right trigger feature/value in `OPTIONAL_SETUP_SETTINGS`. |
| 1000 ms frame returns in ~1 ms, or in ~1000 s | Exposure unit is not µs. Fix the conversion in `_set_exposure_sync` and in the `exposure_ms` read in `_setup_sync`. |
| `incomplete frame (status Incomplete)` | 1394 bandwidth. Look for a `PacketSize`/`BandwidthControl`/`IsoSpeed` feature. Try `--pixel-format Mono8` to halve the data. Report. |
| Stripes or diagonal shearing in the PNG | Stride mismatch: compare `PayloadSize` with width x height x bytes. |

## Step 3: lit capture and bit alignment (A5): needs light

This PC can't turn the light on. Ask the operator to light the field: either run
brightfield from the motion PC (`brightfield_test.py --port COM4 --percents 25
--on-seconds 120 --off-seconds 0`, with that PC's own approval), or open the lid/door so
room light reaches the optics. Then:

```
.venv\Scripts\python tools\celigo_legacy\camera_probe.py capture --exposures-ms 0.1 1 10 100 1000
```

**Pass:** `mean` grows roughly in proportion to exposure until saturation; the PNG shows
an image (field of view, a plate, or a blurry glow); at saturation `max` is ≥ 65000 and
`unused_low_bits` is 2 (14-bit data MSB-aligned in 16 bits).

| Result | Meaning and fix |
|---|---|
| Saturated `max` ≈ 16383, `unused_low_bits` 0 | LSB-aligned. `Celigo.auto_exposure` assumes full scale 65535. Add a left shift by 2 for Mono16 in `_acquire_frame_sync` (with a test), or report it for a decision. |
| Values look byte-swapped (noise everywhere, no structure) | Big-endian Mono16. Byteswap in `_acquire_frame_sync`. |
| `mean` doesn't change with exposure | Exposure isn't being applied: check the `ExposureAuto` note and the feature's write access. |

## Step 4: ROI and Mono8 (A6)

```
.venv\Scripts\python tools\celigo_legacy\camera_probe.py roi --width 1024 --height 1024
.venv\Scripts\python tools\celigo_legacy\camera_probe.py capture --pixel-format Mono8 --exposures-ms 10
```

**Pass:** the ROI frame is `[1024, 1024, 16]` and centered (`x_offset`, `y_offset` ≈ (full −
1024)/2); Mono8 gives `[full_w, full_h, 8]`. If the camera rejects 1024 with `not a
multiple`, retry with a size that matches the increment shown in the error, and report it.

## Step 5 (optional): gain

From step 1, take the gain feature's range. Then run `capture --exposures-ms 10 --gain <mid>`
in the same light as step 3: `gain_applied` ≈ requested, and `mean` higher than at gain 0.

## Report back (paste this, filled in, to the operator)

```
Vimba version / install path:
Camera model, serial, camera_id:
Full geometry (w x h), bit depth:
Exposure feature + unit; range:
Gain feature + unit; range:
AcquisitionMode / TriggerMode values, and what was changed:
Step 2 timings (requested ms -> elapsed s):
Step 3: saturated max, unused_low_bits, endianness OK?:
ROI result and increments:
Code changes made (files, one line each), tests added, pytest result:
Open problems:
```

## After the camera works (not on this PC)

Celigo integration needs the controller and the camera on one PC. Then:
`Celigo(config=load_legacy_config(), camera=AlliedVisionCamera())`, and set
`image_width_pixels`/`image_height_pixels` in `legacy_config._placeholder_calibration` to
the geometry from step 1. `setup()` applies that size as the ROI and rejects mismatched
frames; with the current placeholder `0 x 0`, setup fails. Then calibrate pixel size and
image-to-stage rotation (README, "Future work: imaging", item 3).
