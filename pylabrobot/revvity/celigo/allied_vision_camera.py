"""Async wrapper for Allied Vision IEEE 1394 (FireWire) cameras through Vimba 6's C API.

Older Celigo units use an Allied Vision Pike F-505B on a FireWire interface card. Vimba X
does not support IEEE 1394, so this binds ``VimbaC`` from Vimba 6.x with ``ctypes``. The
SDK's IEEE 1394 transport layer and Allied Vision's 1394 driver must be installed.

Each capture is a ``SingleFrame`` acquisition started by software, so every returned frame
was exposed after the call began. SDK calls are blocking, so public methods run them on
one serialized worker thread with a timeout.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import ctypes
import ctypes.util
import functools
import glob
import os
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple, TypeVar, Union

from pylabrobot.revvity.celigo.camera import CameraError, CameraFrame, _SerializedDaemonExecutor

_T = TypeVar("_T")

VMB_ERROR_SUCCESS = 0
VMB_ERROR_NOT_FOUND = -3
VMB_ERROR_WRONG_TYPE = -10
VMB_ERROR_TIMEOUT = -12
VMB_ERROR_NAMES: Dict[int, str] = {
  -1: "InternalFault",
  -2: "ApiNotStarted",
  -3: "NotFound",
  -4: "BadHandle",
  -5: "DeviceNotOpen",
  -6: "InvalidAccess",
  -7: "BadParameter",
  -8: "StructSize",
  -9: "MoreData",
  -10: "WrongType",
  -11: "InvalidValue",
  -12: "Timeout",
  -13: "Other",
  -14: "Resources",
  -15: "InvalidCall",
  -16: "NoTL",
  -17: "NotImplemented",
  -18: "NotSupported",
  -19: "Incomplete",
  -20: "IO",
}
VMB_ACCESS_MODE_FULL = 1
VMB_FRAME_STATUS_COMPLETE = 0
VMB_FRAME_STATUS_NAMES: Dict[int, str] = {
  0: "Complete",
  -1: "Incomplete",
  -2: "TooSmall",
  -3: "Invalid",
}
VMB_INTERFACE_NAMES: Dict[int, str] = {
  0: "Unknown",
  1: "FireWire",
  2: "Ethernet",
  3: "USB",
  4: "CameraLink",
  5: "CSI2",
}
VMB_FEATURE_DATA_NAMES: Dict[int, str] = {
  0: "Unknown",
  1: "Int",
  2: "Float",
  3: "Enum",
  4: "String",
  5: "Bool",
  6: "Command",
  7: "Raw",
  8: "None",
}

# PixelFormat entry -> (Vimba pixel-format code, bits per stored pixel).
PIXEL_FORMATS: Dict[str, Tuple[int, int]] = {
  "Mono8": (0x01080001, 8),
  "Mono16": (0x01100007, 16),
}

# Exposure features hold microseconds; the first readable name is used.
EXPOSURE_FEATURES = ("ExposureTime", "ExposureTimeAbs")
GAIN_FEATURES = ("Gain", "GainAbs", "GainRaw")
# Optional settings applied at setup when the camera has them: software-started frames
# with fixed exposure and gain.
OPTIONAL_SETUP_SETTINGS = (
  ("TriggerMode", "Off"),
  ("ExposureAuto", "Off"),
  ("GainAuto", "Off"),
)
DEFAULT_LIBRARY_GLOBS = (
  r"C:\Program Files\Allied Vision\Vimba_*\VimbaC\Bin\Win64\VimbaC.dll",
  r"C:\Program Files\Allied Vision Technologies\AVTVimba_*\VimbaC\Bin\Win64\VimbaC.dll",
)


class VimbaError(CameraError):
  """A Vimba SDK call returned an error code."""

  def __init__(self, operation: str, code: int):
    self.operation = operation
    self.code = code
    super().__init__(
      f"{operation} failed (Vimba error {code} {VMB_ERROR_NAMES.get(code, 'Unknown')})"
    )


class _VmbVersionInfo(ctypes.Structure):
  _fields_ = [
    ("major", ctypes.c_uint32),
    ("minor", ctypes.c_uint32),
    ("patch", ctypes.c_uint32),
  ]


class _VmbInterfaceInfo(ctypes.Structure):
  _fields_ = [
    ("interfaceIdString", ctypes.c_char_p),
    ("interfaceType", ctypes.c_uint32),
    ("interfaceName", ctypes.c_char_p),
    ("serialString", ctypes.c_char_p),
    ("permittedAccess", ctypes.c_uint32),
  ]


class _VmbCameraInfo(ctypes.Structure):
  _fields_ = [
    ("cameraIdString", ctypes.c_char_p),
    ("cameraName", ctypes.c_char_p),
    ("modelName", ctypes.c_char_p),
    ("serialString", ctypes.c_char_p),
    ("permittedAccess", ctypes.c_uint32),
    ("interfaceIdString", ctypes.c_char_p),
  ]


class _VmbFeatureInfo(ctypes.Structure):
  _fields_ = [
    ("name", ctypes.c_char_p),
    ("featureDataType", ctypes.c_uint32),
    ("featureFlags", ctypes.c_uint32),
    ("category", ctypes.c_char_p),
    ("displayName", ctypes.c_char_p),
    ("pollingTime", ctypes.c_uint32),
    ("unit", ctypes.c_char_p),
    ("representation", ctypes.c_char_p),
    ("visibility", ctypes.c_uint32),
    ("tooltip", ctypes.c_char_p),
    ("description", ctypes.c_char_p),
    ("sfncNamespace", ctypes.c_char_p),
    ("isStreamable", ctypes.c_char),
    ("hasAffectedFeatures", ctypes.c_char),
    ("hasSelectedFeatures", ctypes.c_char),
  ]


class _VmbFrame(ctypes.Structure):
  _fields_ = [
    ("buffer", ctypes.c_void_p),
    ("bufferSize", ctypes.c_uint32),
    ("context", ctypes.c_void_p * 4),
    ("receiveStatus", ctypes.c_int32),
    ("receiveFlags", ctypes.c_uint32),
    ("imageSize", ctypes.c_uint32),
    ("ancillarySize", ctypes.c_uint32),
    ("pixelFormat", ctypes.c_uint32),
    ("width", ctypes.c_uint32),
    ("height", ctypes.c_uint32),
    ("offsetX", ctypes.c_uint32),
    ("offsetY", ctypes.c_uint32),
    ("frameID", ctypes.c_uint64),
    ("timestamp", ctypes.c_uint64),
  ]


def _text(value: Optional[bytes]) -> str:
  return value.decode("utf-8", "replace") if value else ""


@dataclass(frozen=True)
class VimbaCameraInfo:
  """One camera reported by Vimba discovery."""

  camera_id: str
  name: str
  model: str
  serial: str
  interface_id: str


@dataclass(frozen=True)
class VimbaInterfaceInfo:
  """One transport interface (for example a FireWire bus) reported by Vimba."""

  interface_id: str
  interface_type: str
  name: str


@dataclass(frozen=True)
class VimbaFeature:
  """One camera feature and, when readable, its current value."""

  name: str
  data_type: str
  unit: str
  category: str
  value: Union[int, float, str, bool, None]
  error: str = ""


@dataclass(frozen=True)
class _NumericFeature:
  name: str
  is_float: bool


class AlliedVisionCamera:
  """Allied Vision FireWire camera (for example a Pike F-505B) accessed through Vimba 6.

  Implements :class:`pylabrobot.revvity.celigo.camera.CeligoCamera`. Exposure is in
  milliseconds; gain is the camera's native gain value (dB or raw counts, depending on
  which of :data:`GAIN_FEATURES` the camera exposes, see :attr:`gain_feature`).

  Args:
    camera_id: Vimba camera ID to open; the first camera found when omitted.
    pixel_format: ``"Mono8"`` or ``"Mono16"``.
    sdk_library: Path to ``VimbaC.dll``. Defaults to ``VIMBA_C_LIBRARY``, then
      ``%VIMBA_HOME%``, then the standard Vimba 6 install locations.
    library: A preloaded library object (for tests).
    sdk_call_timeout: Seconds before one blocking SDK operation is abandoned.
    frame_timeout: Seconds to wait for a frame beyond its exposure time.
    discovery_timeout: Seconds to keep polling Vimba for a camera at setup.
  """

  def __init__(
    self,
    camera_id: Optional[str] = None,
    pixel_format: str = "Mono16",
    sdk_library: Optional[str] = None,
    library: Optional[Any] = None,
    sdk_call_timeout: float = 30.0,
    frame_timeout: float = 5.0,
    discovery_timeout: float = 3.0,
  ):
    if pixel_format not in PIXEL_FORMATS:
      raise ValueError(f"pixel_format must be one of {sorted(PIXEL_FORMATS)}")
    if sdk_call_timeout <= 0 or frame_timeout <= 0 or discovery_timeout < 0:
      raise ValueError("timeouts must be positive")
    self.camera_id = camera_id
    self.pixel_format = pixel_format
    self.sdk_library = sdk_library or os.environ.get("VIMBA_C_LIBRARY")
    self._lib = library
    self.sdk_call_timeout = sdk_call_timeout
    self.frame_timeout = frame_timeout
    self.discovery_timeout = discovery_timeout
    self._executor: Optional[_SerializedDaemonExecutor] = _SerializedDaemonExecutor()
    self._pending_cleanup: Optional[concurrent.futures.Future[Any]] = None
    self._started = False
    self._handle: Optional[int] = None
    self._frame: Optional[_VmbFrame] = None
    self._frame_buffer: Optional[ctypes.Array[ctypes.c_char]] = None
    self._capturing = False
    self._exposure_feature: Optional[_NumericFeature] = None
    self._gain_feature: Optional[_NumericFeature] = None
    self.sdk_version = ""
    self.info: Optional[VimbaCameraInfo] = None
    self.setup_notes: List[str] = []
    self.width = 0
    self.height = 0
    self.x_offset = 0
    self.y_offset = 0
    self.bit_depth = PIXEL_FORMATS[pixel_format][1]
    self.exposure_ms = 0.0
    self.gain = 0.0

  @property
  def is_open(self) -> bool:
    return self._handle is not None and self._pending_cleanup is None

  @property
  def exposure_feature(self) -> str:
    """Name of the camera feature used for exposure, after setup."""
    return self._exposure_feature.name if self._exposure_feature else ""

  @property
  def gain_feature(self) -> str:
    """Name of the camera feature used for gain, after setup."""
    return self._gain_feature.name if self._gain_feature else ""

  @functools.cached_property
  def _lock(self) -> asyncio.Lock:
    return asyncio.Lock()

  # -- worker thread -----------------------------------------------------------

  def _queue_deferred_close(
    self, executor: _SerializedDaemonExecutor
  ) -> concurrent.futures.Future[Any]:
    cleanup = executor.submit(self._stop_sync)
    self._pending_cleanup = cleanup

    def shutdown_worker(_future: concurrent.futures.Future[Any]) -> None:
      executor.shutdown()
      if self._executor is executor:
        self._executor = None

    cleanup.add_done_callback(shutdown_worker)
    return cleanup

  async def _run_blocking(self, function: Callable[..., _T], *args: Any) -> _T:
    """Run one SDK operation on the serialized worker, with the SDK call timeout.

    A native call cannot be cancelled, so on timeout or cancellation the close is queued
    behind it and the camera refuses further calls until that close completes.
    """
    pending_cleanup = self._pending_cleanup
    if pending_cleanup is not None:
      if not pending_cleanup.done():
        raise CameraError(
          "A timed-out Vimba call is still running; the camera is poisoned until its "
          "deferred close completes"
        )
      pending_cleanup.result()
      self._pending_cleanup = None
    if self._executor is None:
      self._executor = _SerializedDaemonExecutor()
    executor = self._executor
    worker = executor.submit(functools.partial(function, *args))
    deadline = asyncio.get_running_loop().time() + self.sdk_call_timeout
    try:
      while not worker.done():
        remaining = deadline - asyncio.get_running_loop().time()
        if remaining <= 0:
          raise asyncio.TimeoutError
        await asyncio.sleep(min(0.005, remaining))
      return worker.result()
    except asyncio.CancelledError:
      self._queue_deferred_close(executor)
      raise
    except asyncio.TimeoutError as exc:
      self._queue_deferred_close(executor)
      raise CameraError(
        f"Vimba SDK call exceeded {self.sdk_call_timeout:g} second timeout; camera close "
        "is queued behind the native call"
      ) from exc

  def _shutdown_executor(self) -> None:
    if self._pending_cleanup is None and self._executor is not None:
      self._executor.shutdown()
      self._executor = None

  # -- library -----------------------------------------------------------------

  def _library_candidates(self) -> List[str]:
    candidates = [self.sdk_library] if self.sdk_library else []
    vimba_home = os.environ.get("VIMBA_HOME")
    if vimba_home:
      candidates.append(os.path.join(vimba_home, "VimbaC", "Bin", "Win64", "VimbaC.dll"))
    for pattern in DEFAULT_LIBRARY_GLOBS:
      candidates.extend(sorted(glob.glob(pattern), reverse=True))
    discovered = ctypes.util.find_library("VimbaC")
    if discovered:
      candidates.append(discovered)
    candidates.extend(["VimbaC.dll", "libVimbaC.so"])
    return list(dict.fromkeys(candidates))

  def _load_library(self) -> Any:
    if self._lib is None:
      errors = []
      for path in self._library_candidates():
        try:
          self._lib = ctypes.CDLL(path)
          break
        except OSError as exc:
          errors.append(f"{path}: {exc}")
      if self._lib is None:
        raise CameraError(
          "Could not load the Vimba 6 C library (VimbaC.dll, 64-bit); pass sdk_library= "
          f"or set VIMBA_C_LIBRARY. Tried: {'; '.join(errors)}"
        )
    lib = self._lib

    # Bind each signature on the attribute-resolved function object that is called below.
    def set_signature(function: Any, argtypes: List[Any]) -> None:
      try:
        function.restype = ctypes.c_int32
        function.argtypes = argtypes
      except AttributeError:
        return

    handle = ctypes.c_void_p
    name = ctypes.c_char_p
    u32 = ctypes.c_uint32
    p_u32 = ctypes.POINTER(ctypes.c_uint32)
    p_i64 = ctypes.POINTER(ctypes.c_int64)
    p_double = ctypes.POINTER(ctypes.c_double)
    p_frame = ctypes.POINTER(_VmbFrame)
    try:
      set_signature(lib.VmbVersionQuery, [ctypes.POINTER(_VmbVersionInfo), u32])
      set_signature(lib.VmbStartup, [])
      set_signature(lib.VmbShutdown, [])
      set_signature(lib.VmbInterfacesList, [ctypes.POINTER(_VmbInterfaceInfo), u32, p_u32, u32])
      set_signature(lib.VmbCamerasList, [ctypes.POINTER(_VmbCameraInfo), u32, p_u32, u32])
      set_signature(lib.VmbCameraOpen, [name, u32, ctypes.POINTER(ctypes.c_void_p)])
      set_signature(lib.VmbCameraClose, [handle])
      set_signature(lib.VmbFeaturesList, [handle, ctypes.POINTER(_VmbFeatureInfo), u32, p_u32, u32])
      set_signature(lib.VmbFeatureIntGet, [handle, name, p_i64])
      set_signature(lib.VmbFeatureIntSet, [handle, name, ctypes.c_int64])
      set_signature(lib.VmbFeatureIntRangeQuery, [handle, name, p_i64, p_i64])
      set_signature(lib.VmbFeatureIntIncrementQuery, [handle, name, p_i64])
      set_signature(lib.VmbFeatureFloatGet, [handle, name, p_double])
      set_signature(lib.VmbFeatureFloatSet, [handle, name, ctypes.c_double])
      set_signature(lib.VmbFeatureEnumGet, [handle, name, ctypes.POINTER(ctypes.c_char_p)])
      set_signature(lib.VmbFeatureEnumSet, [handle, name, name])
      set_signature(lib.VmbFeatureBoolGet, [handle, name, ctypes.POINTER(ctypes.c_char)])
      set_signature(lib.VmbFeatureStringGet, [handle, name, ctypes.c_char_p, u32, p_u32])
      set_signature(lib.VmbFeatureCommandRun, [handle, name])
      set_signature(lib.VmbFrameAnnounce, [handle, p_frame, u32])
      set_signature(lib.VmbFrameRevokeAll, [handle])
      set_signature(lib.VmbCaptureStart, [handle])
      set_signature(lib.VmbCaptureEnd, [handle])
      set_signature(lib.VmbCaptureFrameQueue, [handle, p_frame, ctypes.c_void_p])
      set_signature(lib.VmbCaptureFrameWait, [handle, p_frame, u32])
      set_signature(lib.VmbCaptureQueueFlush, [handle])
    except AttributeError as exc:
      raise CameraError(f"Vimba SDK is missing a required export: {exc}") from exc
    return lib

  def _require_library(self) -> Any:
    if self._lib is None:
      raise CameraError("Vimba SDK is not loaded; call setup() first")
    return self._lib

  def _require_handle(self) -> int:
    if self._handle is None:
      raise CameraError("Camera is not open; call setup() first")
    return self._handle

  @staticmethod
  def _check(code: int, operation: str) -> None:
    if code != VMB_ERROR_SUCCESS:
      raise VimbaError(operation, int(code))

  # -- features ----------------------------------------------------------------

  def _get_int_sync(self, name: str) -> int:
    value = ctypes.c_int64()
    self._check(
      self._require_library().VmbFeatureIntGet(
        self._require_handle(), name.encode(), ctypes.byref(value)
      ),
      f"read {name}",
    )
    return int(value.value)

  def _set_int_sync(self, name: str, value: int) -> None:
    self._check(
      self._require_library().VmbFeatureIntSet(self._require_handle(), name.encode(), value),
      f"set {name}={value}",
    )

  def _get_int_range_sync(self, name: str) -> Tuple[int, int]:
    minimum = ctypes.c_int64()
    maximum = ctypes.c_int64()
    self._check(
      self._require_library().VmbFeatureIntRangeQuery(
        self._require_handle(), name.encode(), ctypes.byref(minimum), ctypes.byref(maximum)
      ),
      f"read {name} range",
    )
    return int(minimum.value), int(maximum.value)

  def _get_int_increment_sync(self, name: str) -> int:
    increment = ctypes.c_int64()
    code = self._require_library().VmbFeatureIntIncrementQuery(
      self._require_handle(), name.encode(), ctypes.byref(increment)
    )
    return max(1, int(increment.value)) if code == VMB_ERROR_SUCCESS else 1

  def _get_float_sync(self, name: str) -> float:
    value = ctypes.c_double()
    self._check(
      self._require_library().VmbFeatureFloatGet(
        self._require_handle(), name.encode(), ctypes.byref(value)
      ),
      f"read {name}",
    )
    return float(value.value)

  def _set_float_sync(self, name: str, value: float) -> None:
    self._check(
      self._require_library().VmbFeatureFloatSet(self._require_handle(), name.encode(), value),
      f"set {name}={value:g}",
    )

  def _get_enum_sync(self, name: str) -> str:
    value = ctypes.c_char_p()
    self._check(
      self._require_library().VmbFeatureEnumGet(
        self._require_handle(), name.encode(), ctypes.byref(value)
      ),
      f"read {name}",
    )
    return _text(value.value)

  def _set_enum_sync(self, name: str, value: str) -> None:
    self._check(
      self._require_library().VmbFeatureEnumSet(
        self._require_handle(), name.encode(), value.encode()
      ),
      f"set {name}={value}",
    )

  def _run_command_sync(self, name: str) -> int:
    return int(self._require_library().VmbFeatureCommandRun(self._require_handle(), name.encode()))

  def _resolve_numeric_feature_sync(self, candidates: Tuple[str, ...]) -> _NumericFeature:
    """Return the first candidate feature that reads as a float or an integer."""
    lib = self._require_library()
    handle = self._require_handle()
    for name in candidates:
      float_value = ctypes.c_double()
      code = int(lib.VmbFeatureFloatGet(handle, name.encode(), ctypes.byref(float_value)))
      if code == VMB_ERROR_SUCCESS:
        return _NumericFeature(name, is_float=True)
      if code == VMB_ERROR_WRONG_TYPE:
        int_value = ctypes.c_int64()
        code = int(lib.VmbFeatureIntGet(handle, name.encode(), ctypes.byref(int_value)))
        if code == VMB_ERROR_SUCCESS:
          return _NumericFeature(name, is_float=False)
    raise CameraError(f"Camera has none of the features {', '.join(candidates)}")

  def _get_numeric_sync(self, feature: _NumericFeature) -> float:
    if feature.is_float:
      return self._get_float_sync(feature.name)
    return float(self._get_int_sync(feature.name))

  def _set_numeric_sync(self, feature: _NumericFeature, value: float) -> float:
    if feature.is_float:
      self._set_float_sync(feature.name, value)
    else:
      self._set_int_sync(feature.name, int(round(value)))
    return self._get_numeric_sync(feature)

  def _read_feature_value_sync(
    self, name: str, data_type: int
  ) -> Union[int, float, str, bool, None]:
    lib = self._require_library()
    handle = self._require_handle()
    if data_type == 1:
      return self._get_int_sync(name)
    if data_type == 2:
      return self._get_float_sync(name)
    if data_type == 3:
      return self._get_enum_sync(name)
    if data_type == 5:
      flag = ctypes.c_char()
      self._check(lib.VmbFeatureBoolGet(handle, name.encode(), ctypes.byref(flag)), f"read {name}")
      return flag.value != b"\x00"
    if data_type == 4:
      buffer = ctypes.create_string_buffer(512)
      filled = ctypes.c_uint32()
      self._check(
        lib.VmbFeatureStringGet(handle, name.encode(), buffer, 512, ctypes.byref(filled)),
        f"read {name}",
      )
      return _text(buffer.value)
    return None

  def _list_features_sync(self) -> List[VimbaFeature]:
    lib = self._require_library()
    handle = self._require_handle()
    count = ctypes.c_uint32()
    size = ctypes.sizeof(_VmbFeatureInfo)
    self._check(lib.VmbFeaturesList(handle, None, 0, ctypes.byref(count), size), "count features")
    infos = (_VmbFeatureInfo * max(1, count.value))()
    self._check(
      lib.VmbFeaturesList(handle, infos, count.value, ctypes.byref(count), size), "list features"
    )
    features = []
    for info in infos[: count.value]:
      name = _text(info.name)
      value: Union[int, float, str, bool, None] = None
      error = ""
      try:
        value = self._read_feature_value_sync(name, int(info.featureDataType))
      except CameraError as exc:
        error = str(exc)
      features.append(
        VimbaFeature(
          name=name,
          data_type=VMB_FEATURE_DATA_NAMES.get(int(info.featureDataType), "Unknown"),
          unit=_text(info.unit),
          category=_text(info.category),
          value=value,
          error=error,
        )
      )
    return features

  async def list_features(self) -> List[VimbaFeature]:
    """Return every camera feature with its type, unit, and current value."""
    async with self._lock:
      return await self._run_blocking(self._list_features_sync)

  # -- discovery and lifecycle -------------------------------------------------

  def _list_interfaces_sync(self) -> List[VimbaInterfaceInfo]:
    lib = self._require_library()
    count = ctypes.c_uint32()
    size = ctypes.sizeof(_VmbInterfaceInfo)
    self._check(lib.VmbInterfacesList(None, 0, ctypes.byref(count), size), "count interfaces")
    infos = (_VmbInterfaceInfo * max(1, count.value))()
    self._check(
      lib.VmbInterfacesList(infos, count.value, ctypes.byref(count), size), "list interfaces"
    )
    return [
      VimbaInterfaceInfo(
        interface_id=_text(info.interfaceIdString),
        interface_type=VMB_INTERFACE_NAMES.get(int(info.interfaceType), "Unknown"),
        name=_text(info.interfaceName),
      )
      for info in infos[: count.value]
    ]

  def _list_cameras_sync(self) -> List[VimbaCameraInfo]:
    lib = self._require_library()
    count = ctypes.c_uint32()
    size = ctypes.sizeof(_VmbCameraInfo)
    self._check(lib.VmbCamerasList(None, 0, ctypes.byref(count), size), "count cameras")
    infos = (_VmbCameraInfo * max(1, count.value))()
    self._check(lib.VmbCamerasList(infos, count.value, ctypes.byref(count), size), "list cameras")
    return [
      VimbaCameraInfo(
        camera_id=_text(info.cameraIdString),
        name=_text(info.cameraName),
        model=_text(info.modelName),
        serial=_text(info.serialString),
        interface_id=_text(info.interfaceIdString),
      )
      for info in infos[: count.value]
    ]

  def _discover_camera_sync(self) -> VimbaCameraInfo:
    """Poll discovery until the requested (or any) camera appears."""
    deadline = time.monotonic() + self.discovery_timeout
    while True:
      cameras = self._list_cameras_sync()
      for camera in cameras:
        if self.camera_id is None or camera.camera_id == self.camera_id:
          return camera
      if time.monotonic() >= deadline:
        break
      time.sleep(0.25)
    interfaces = ", ".join(
      f"{interface.interface_type}:{interface.name or interface.interface_id}"
      for interface in self._list_interfaces_sync()
    )
    found = ", ".join(camera.camera_id for camera in cameras) or "none"
    wanted = f"camera {self.camera_id}" if self.camera_id else "camera"
    raise CameraError(
      f"Vimba found no {wanted} (cameras: {found}; interfaces: {interfaces or 'none'}). "
      "No FireWire interface means the Vimba 1394 transport layer or Allied Vision 1394 "
      "driver is not installed for this card."
    )

  def _read_geometry_sync(self) -> None:
    pixel_format = self._get_enum_sync("PixelFormat")
    if pixel_format not in PIXEL_FORMATS:
      raise CameraError(
        f"Unsupported pixel format {pixel_format}; supported: {sorted(PIXEL_FORMATS)}"
      )
    self.bit_depth = PIXEL_FORMATS[pixel_format][1]
    self.width = self._get_int_sync("Width")
    self.height = self._get_int_sync("Height")
    self.x_offset = self._get_int_sync("OffsetX")
    self.y_offset = self._get_int_sync("OffsetY")
    if self.width <= 0 or self.height <= 0:
      raise CameraError(f"Camera reported invalid dimensions {self.width}x{self.height}")

  def _start_capture_sync(self) -> None:
    lib = self._require_library()
    handle = self._require_handle()
    payload = self._get_int_sync("PayloadSize")
    self._frame_buffer = ctypes.create_string_buffer(payload)
    frame = _VmbFrame()
    frame.buffer = ctypes.cast(self._frame_buffer, ctypes.c_void_p)
    frame.bufferSize = payload
    self._frame = frame
    self._check(
      lib.VmbFrameAnnounce(handle, ctypes.byref(frame), ctypes.sizeof(_VmbFrame)), "announce frame"
    )
    self._check(lib.VmbCaptureStart(handle), "start capture engine")
    self._capturing = True

  def _end_capture_sync(self) -> None:
    lib = self._require_library()
    handle = self._require_handle()
    if self._capturing:
      self._run_command_sync("AcquisitionStop")
      lib.VmbCaptureEnd(handle)
      lib.VmbCaptureQueueFlush(handle)
      self._capturing = False
    if self._frame is not None:
      lib.VmbFrameRevokeAll(handle)
      self._frame = None
      self._frame_buffer = None

  def _apply_optional_settings_sync(self) -> None:
    for name, value in OPTIONAL_SETUP_SETTINGS:
      try:
        self._set_enum_sync(name, value)
        self.setup_notes.append(f"{name}={value}")
      except VimbaError as exc:
        self.setup_notes.append(f"{name} not set: {exc}")

  def _setup_sync(self) -> None:
    lib = self._load_library()
    self.setup_notes = []
    version = _VmbVersionInfo()
    if lib.VmbVersionQuery(ctypes.byref(version), ctypes.sizeof(_VmbVersionInfo)) == 0:
      self.sdk_version = f"{version.major}.{version.minor}.{version.patch}"
    self._check(lib.VmbStartup(), "start Vimba")
    self._started = True
    try:
      self.info = self._discover_camera_sync()
      handle = ctypes.c_void_p()
      self._check(
        lib.VmbCameraOpen(self.info.camera_id.encode(), VMB_ACCESS_MODE_FULL, ctypes.byref(handle)),
        f"open camera {self.info.camera_id}",
      )
      if not handle.value:
        raise CameraError(f"Vimba returned a null handle for {self.info.camera_id}")
      self._handle = handle.value
      self._set_enum_sync("AcquisitionMode", "SingleFrame")
      self._apply_optional_settings_sync()
      self._set_enum_sync("PixelFormat", self.pixel_format)
      self._exposure_feature = self._resolve_numeric_feature_sync(EXPOSURE_FEATURES)
      self._gain_feature = self._resolve_numeric_feature_sync(GAIN_FEATURES)
      self._read_geometry_sync()
      self.exposure_ms = self._get_numeric_sync(self._exposure_feature) / 1000.0
      self.gain = self._get_numeric_sync(self._gain_feature)
      self._start_capture_sync()
    except Exception:
      self._stop_sync()
      raise

  async def setup(self) -> None:
    """Start Vimba, open the camera, and prepare software-started single-frame capture."""
    async with self._lock:
      if not self.is_open:
        try:
          await self._run_blocking(self._setup_sync)
        except BaseException:
          self._shutdown_executor()
          raise

  def _stop_sync(self) -> None:
    lib = self._lib
    if lib is None:
      return
    if self._handle is not None:
      try:
        self._end_capture_sync()
      finally:
        lib.VmbCameraClose(self._handle)
        self._handle = None
    if self._started:
      lib.VmbShutdown()
      self._started = False

  async def stop(self) -> None:
    """Stop capture, close the camera, and shut Vimba down."""
    async with self._lock:
      try:
        await self._run_blocking(self._stop_sync)
      finally:
        self._shutdown_executor()

  # -- settings ----------------------------------------------------------------

  def _set_exposure_sync(self, exposure_ms: float) -> float:
    if self._exposure_feature is None:
      raise CameraError("Camera is not open; call setup() first")
    return self._set_numeric_sync(self._exposure_feature, exposure_ms * 1000.0) / 1000.0

  async def set_exposure(self, exposure_ms: float) -> float:
    """Set the exposure time in milliseconds; returns the value the camera applied."""
    if exposure_ms <= 0:
      raise ValueError("exposure_ms must be positive")
    async with self._lock:
      self.exposure_ms = await self._run_blocking(self._set_exposure_sync, exposure_ms)
      return self.exposure_ms

  def _set_gain_sync(self, gain: float) -> float:
    if self._gain_feature is None:
      raise CameraError("Camera is not open; call setup() first")
    return self._set_numeric_sync(self._gain_feature, gain)

  async def set_gain(self, gain: float) -> float:
    """Set the camera's native gain; returns the value the camera applied."""
    if gain < 0:
      raise ValueError("gain must be non-negative")
    async with self._lock:
      self.gain = await self._run_blocking(self._set_gain_sync, gain)
      return self.gain

  def _set_frame_format_sync(
    self,
    width: int,
    height: int,
    x_offset: Optional[int],
    y_offset: Optional[int],
  ) -> Tuple[int, int]:
    self._end_capture_sync()
    try:
      self._set_int_sync("OffsetX", 0)
      self._set_int_sync("OffsetY", 0)
      _, maximum_width = self._get_int_range_sync("Width")
      _, maximum_height = self._get_int_range_sync("Height")
      if width > maximum_width or height > maximum_height:
        raise CameraError(
          f"Requested camera format {width}x{height} exceeds sensor "
          f"{maximum_width}x{maximum_height}"
        )
      for name, value in (("Width", width), ("Height", height)):
        increment = self._get_int_increment_sync(name)
        if value % increment:
          raise CameraError(f"{name} {value} is not a multiple of the camera's {increment}")
      self._set_int_sync("Width", width)
      self._set_int_sync("Height", height)
      x_step = self._get_int_increment_sync("OffsetX")
      y_step = self._get_int_increment_sync("OffsetY")
      target_x = (maximum_width - width) // 2 // x_step * x_step if x_offset is None else x_offset
      target_y = (maximum_height - height) // 2 // y_step * y_step if y_offset is None else y_offset
      if target_x < 0 or target_y < 0:
        raise ValueError("camera offsets must be non-negative")
      self._set_int_sync("OffsetX", target_x)
      self._set_int_sync("OffsetY", target_y)
      self._read_geometry_sync()
      if (self.width, self.height) != (width, height):
        raise CameraError(f"Camera accepted {self.width}x{self.height}, not {width}x{height}")
    finally:
      self._start_capture_sync()
    return self.width, self.height

  async def set_frame_format(
    self,
    width: int,
    height: int,
    x_offset: Optional[int] = None,
    y_offset: Optional[int] = None,
  ) -> Tuple[int, int]:
    """Set and verify a camera ROI, centered on the sensor when offsets are omitted."""
    if width <= 0 or height <= 0:
      raise ValueError("camera width and height must be positive")
    async with self._lock:
      return await self._run_blocking(
        self._set_frame_format_sync, width, height, x_offset, y_offset
      )

  # -- capture -----------------------------------------------------------------

  def _acquire_frame_sync(self) -> bytes:
    lib = self._require_library()
    handle = self._require_handle()
    frame = self._frame
    if frame is None or not self._capturing:
      raise CameraError("Capture engine is not running; call setup() first")
    frame.receiveStatus = -3
    self._check(lib.VmbCaptureFrameQueue(handle, ctypes.byref(frame), None), "queue frame")
    self._check(self._run_command_sync("AcquisitionStart"), "start acquisition")
    timeout_ms = int(self.exposure_ms + self.frame_timeout * 1000.0)
    code = int(lib.VmbCaptureFrameWait(handle, ctypes.byref(frame), timeout_ms))
    self._run_command_sync("AcquisitionStop")
    if code != VMB_ERROR_SUCCESS:
      lib.VmbCaptureQueueFlush(handle)
      if code == VMB_ERROR_TIMEOUT:
        raise CameraError(
          f"No frame within {timeout_ms} ms; the camera may be waiting for a hardware "
          "trigger (check TriggerMode in setup_notes)"
        )
      raise VimbaError("wait for frame", code)
    if frame.receiveStatus != VMB_FRAME_STATUS_COMPLETE:
      status = VMB_FRAME_STATUS_NAMES.get(int(frame.receiveStatus), str(frame.receiveStatus))
      raise CameraError(f"Camera delivered an incomplete frame (status {status})")
    expected_format = PIXEL_FORMATS[self.pixel_format][0]
    if (frame.width, frame.height, frame.pixelFormat) != (
      self.width,
      self.height,
      expected_format,
    ):
      raise CameraError(
        f"Frame is {frame.width}x{frame.height} format {frame.pixelFormat:#x}; expected "
        f"{self.width}x{self.height} format {expected_format:#x}"
      )
    byte_count = self.width * self.height * (self.bit_depth // 8)
    if frame.imageSize < byte_count:
      raise CameraError(f"Frame holds {frame.imageSize} bytes; {byte_count} are required")
    return ctypes.string_at(frame.buffer, byte_count)

  def _capture_sync(self, flush_frames: int) -> CameraFrame:
    data = b""
    for _ in range(flush_frames + 1):
      data = self._acquire_frame_sync()
    return CameraFrame(
      data=data,
      width=self.width,
      height=self.height,
      bit_depth=self.bit_depth,
      exposure_ms=self.exposure_ms,
      gain=self.gain,
      captured_at=time.time(),
    )

  async def capture(self, flush_frames: int = 2) -> CameraFrame:
    """Expose and return one frame, after acquiring and discarding ``flush_frames``."""
    if flush_frames < 0:
      raise ValueError("flush_frames must be non-negative")
    async with self._lock:
      return await self._run_blocking(self._capture_sync, flush_frames)
