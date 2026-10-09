import ctypes
import unittest
from typing import Any, Dict, List, Optional, Union

from pylabrobot.revvity.celigo.allied_vision_camera import (
  PIXEL_FORMATS,
  VMB_ERROR_NOT_FOUND,
  VMB_ERROR_TIMEOUT,
  VMB_ERROR_WRONG_TYPE,
  AlliedVisionCamera,
)
from pylabrobot.revvity.celigo.camera import CameraError

SENSOR_WIDTH = 8
SENSOR_HEIGHT = 6


class _FakeVimbaLibrary:
  """In-memory stand-in for ``VimbaC`` with one FireWire camera."""

  def __init__(self, cameras: int = 1):
    self.cameras = cameras
    self.started = False
    self.open_handle: Optional[int] = None
    self.capturing = False
    self.announced: Optional[Any] = None
    self.queued = False
    self.wait_result = 0
    self.frame_status = 0
    self.acquisitions = 0
    self.calls: List[str] = []
    self.floats: Dict[str, float] = {"ExposureTime": 10000.0}
    self.ints: Dict[str, int] = {
      "GainRaw": 0,
      "Width": SENSOR_WIDTH,
      "Height": SENSOR_HEIGHT,
      "OffsetX": 0,
      "OffsetY": 0,
    }
    self.enums: Dict[str, str] = {
      "AcquisitionMode": "Continuous",
      "PixelFormat": "Mono8",
      "TriggerMode": "On",
    }

  def VmbVersionQuery(self, version, _size):
    version._obj.major, version._obj.minor, version._obj.patch = 1, 9, 0
    return 0

  def VmbStartup(self):
    self.started = True
    return 0

  def VmbShutdown(self):
    self.started = False

  def VmbInterfacesList(self, infos, _length, count, _size):
    count._obj.value = 1
    if infos is not None:
      infos[0].interfaceIdString = b"1394-0"
      infos[0].interfaceType = 1
      infos[0].interfaceName = b"1394 bus"
    return 0

  def VmbCamerasList(self, infos, _length, count, _size):
    count._obj.value = self.cameras
    if infos is not None and self.cameras:
      infos[0].cameraIdString = b"DEV_000A4701"
      infos[0].modelName = b"Pike F-505B"
      infos[0].serialString = b"123"
    return 0

  def VmbCameraOpen(self, camera_id, _mode, handle):
    self.calls.append(f"open {camera_id.decode()}")
    handle._obj.value = 42
    self.open_handle = 42
    return 0

  def VmbCameraClose(self, _handle):
    self.open_handle = None
    return 0

  def VmbFeaturesList(self, _handle, _infos, _length, count, _size):
    count._obj.value = 0
    return 0

  def VmbFeatureBoolGet(self, _handle, _name, _value):
    return VMB_ERROR_NOT_FOUND

  def VmbFeatureStringGet(self, _handle, _name, _buffer, _size, _filled):
    return VMB_ERROR_NOT_FOUND

  def VmbFeatureFloatGet(self, _handle, name, value):
    key = name.decode()
    if key in self.floats:
      value._obj.value = self.floats[key]
      return 0
    return VMB_ERROR_WRONG_TYPE if key in self.ints else VMB_ERROR_NOT_FOUND

  def VmbFeatureFloatSet(self, _handle, name, value):
    self.floats[name.decode()] = value
    return 0

  def _int_value(self, key: str) -> int:
    if key == "PayloadSize":
      bytes_per_pixel = PIXEL_FORMATS[self.enums["PixelFormat"]][1] // 8
      return self.ints["Width"] * self.ints["Height"] * bytes_per_pixel
    return self.ints[key]

  def VmbFeatureIntGet(self, _handle, name, value):
    key = name.decode()
    if key not in self.ints and key != "PayloadSize":
      return VMB_ERROR_WRONG_TYPE if key in self.floats else VMB_ERROR_NOT_FOUND
    value._obj.value = self._int_value(key)
    return 0

  def VmbFeatureIntSet(self, _handle, name, value):
    self.calls.append(f"{name.decode()}={value}")
    self.ints[name.decode()] = value
    return 0

  def VmbFeatureIntRangeQuery(self, _handle, name, minimum, maximum):
    sensor = {"Width": SENSOR_WIDTH, "Height": SENSOR_HEIGHT}[name.decode()]
    offset = self.ints["OffsetX" if name == b"Width" else "OffsetY"]
    minimum._obj.value, maximum._obj.value = 2, sensor - offset
    return 0

  def VmbFeatureIntIncrementQuery(self, _handle, name, value):
    value._obj.value = 2
    return 0

  def VmbFeatureEnumGet(self, _handle, name, value):
    key = name.decode()
    if key not in self.enums:
      return VMB_ERROR_NOT_FOUND
    value._obj.value = self.enums[key].encode()
    return 0

  def VmbFeatureEnumSet(self, _handle, name, value):
    key = name.decode()
    if key not in self.enums:
      return VMB_ERROR_NOT_FOUND
    self.enums[key] = value.decode()
    return 0

  def VmbFeatureCommandRun(self, _handle, name):
    self.calls.append(name.decode())
    if name == b"AcquisitionStart":
      self.acquisitions += 1
    return 0

  def VmbFrameAnnounce(self, _handle, frame, _size):
    self.announced = frame._obj
    return 0

  def VmbFrameRevokeAll(self, _handle):
    self.announced = None
    return 0

  def VmbCaptureStart(self, _handle):
    self.calls.append("capture start")
    self.capturing = True
    return 0

  def VmbCaptureEnd(self, _handle):
    self.calls.append("capture end")
    self.capturing = False
    return 0

  def VmbCaptureQueueFlush(self, _handle):
    self.queued = False
    return 0

  def VmbCaptureFrameQueue(self, _handle, _frame, _callback):
    self.queued = True
    return 0

  def VmbCaptureFrameWait(self, _handle, frame, _timeout):
    if self.wait_result:
      return self.wait_result
    target = frame._obj
    pixel_format, bits = PIXEL_FORMATS[self.enums["PixelFormat"]]
    size = self.ints["Width"] * self.ints["Height"] * bits // 8
    ctypes.memmove(target.buffer, bytes(range(size)), size)
    target.receiveStatus = self.frame_status
    target.imageSize = size
    target.pixelFormat = pixel_format
    target.width = self.ints["Width"]
    target.height = self.ints["Height"]
    self.queued = False
    return 0


def _camera(library: _FakeVimbaLibrary, pixel_format: str = "Mono8") -> AlliedVisionCamera:
  return AlliedVisionCamera(library=library, pixel_format=pixel_format, discovery_timeout=0)


class TestAlliedVisionCamera(unittest.IsolatedAsyncioTestCase):
  async def test_setup_configures_single_frame_software_capture(self):
    library = _FakeVimbaLibrary()
    camera = _camera(library, pixel_format="Mono16")
    await camera.setup()
    self.assertTrue(camera.is_open)
    self.assertEqual(camera.info.model if camera.info else "", "Pike F-505B")
    self.assertEqual(camera.sdk_version, "1.9.0")
    self.assertEqual(library.enums["AcquisitionMode"], "SingleFrame")
    self.assertEqual(library.enums["TriggerMode"], "Off")
    self.assertEqual(library.enums["PixelFormat"], "Mono16")
    self.assertIn("ExposureAuto not set", " ".join(camera.setup_notes))
    self.assertEqual((camera.width, camera.height, camera.bit_depth), (8, 6, 16))
    self.assertEqual((camera.exposure_feature, camera.gain_feature), ("ExposureTime", "GainRaw"))
    self.assertAlmostEqual(camera.exposure_ms, 10.0)
    self.assertTrue(library.capturing)
    await camera.stop()
    self.assertIsNone(library.open_handle)
    self.assertFalse(library.started)
    self.assertIsNone(camera._executor)

  async def test_exposure_is_converted_and_integer_gain_is_rounded(self):
    library = _FakeVimbaLibrary()
    camera = _camera(library)
    await camera.setup()
    self.assertAlmostEqual(await camera.set_exposure(2.5), 2.5)
    self.assertAlmostEqual(library.floats["ExposureTime"], 2500.0)
    self.assertEqual(await camera.set_gain(3.6), 4.0)
    self.assertEqual(library.ints["GainRaw"], 4)
    await camera.stop()

  async def test_capture_returns_the_frame_after_flushed_acquisitions(self):
    library = _FakeVimbaLibrary()
    camera = _camera(library, pixel_format="Mono16")
    await camera.setup()
    frame = await camera.capture(flush_frames=2)
    self.assertEqual(library.acquisitions, 3)
    self.assertEqual((frame.width, frame.height, frame.bit_depth), (8, 6, 16))
    self.assertEqual(frame.data, bytes(range(96)))
    await camera.stop()

  async def test_centered_roi_resets_offsets_and_restarts_capture(self):
    library = _FakeVimbaLibrary()
    camera = _camera(library)
    await camera.setup()
    library.calls.clear()
    self.assertEqual(await camera.set_frame_format(4, 2), (4, 2))
    self.assertEqual((camera.x_offset, camera.y_offset), (2, 2))
    self.assertEqual(
      library.calls,
      [
        "AcquisitionStop",
        "capture end",
        "OffsetX=0",
        "OffsetY=0",
        "Width=4",
        "Height=2",
        "OffsetX=2",
        "OffsetY=2",
        "capture start",
      ],
    )
    frame = await camera.capture(flush_frames=0)
    self.assertEqual(len(frame.data), 8)
    await camera.stop()

  async def test_roi_rejects_sizes_off_the_increment(self):
    library = _FakeVimbaLibrary()
    camera = _camera(library)
    await camera.setup()
    with self.assertRaisesRegex(CameraError, "not a multiple"):
      await camera.set_frame_format(3, 2)
    self.assertTrue(library.capturing)
    await camera.stop()

  async def test_missing_camera_reports_interfaces_and_shuts_down(self):
    library = _FakeVimbaLibrary(cameras=0)
    camera = _camera(library)
    with self.assertRaisesRegex(CameraError, "FireWire:1394 bus"):
      await camera.setup()
    self.assertFalse(library.started)
    self.assertFalse(camera.is_open)

  async def test_frame_timeout_points_at_the_trigger_mode(self):
    library = _FakeVimbaLibrary()
    camera = _camera(library)
    await camera.setup()
    library.wait_result = VMB_ERROR_TIMEOUT
    with self.assertRaisesRegex(CameraError, "hardware trigger"):
      await camera.capture(flush_frames=0)
    self.assertFalse(library.queued)
    await camera.stop()

  async def test_incomplete_frame_is_rejected(self):
    library = _FakeVimbaLibrary()
    camera = _camera(library)
    await camera.setup()
    library.frame_status = -1
    with self.assertRaisesRegex(CameraError, "Incomplete"):
      await camera.capture(flush_frames=0)
    await camera.stop()

  async def test_list_features_reads_typed_values(self):
    library = _FakeVimbaLibrary()

    def features_list(_handle, infos, _length, count, _size):
      count._obj.value = 2
      if infos is not None:
        infos[0].name, infos[0].featureDataType, infos[0].unit = b"ExposureTime", 2, b"us"
        infos[1].name, infos[1].featureDataType = b"PixelFormat", 3
      return 0

    library.VmbFeaturesList = features_list  # type: ignore[attr-defined]
    camera = _camera(library)
    await camera.setup()
    features = await camera.list_features()
    values: Dict[str, Union[int, float, str, bool, None]] = {f.name: f.value for f in features}
    self.assertEqual(values, {"ExposureTime": 10000.0, "PixelFormat": "Mono8"})
    self.assertEqual(features[0].unit, "us")
    await camera.stop()


if __name__ == "__main__":
  unittest.main()
