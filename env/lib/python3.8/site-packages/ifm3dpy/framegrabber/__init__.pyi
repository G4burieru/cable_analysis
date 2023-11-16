"""Provides an implementation of the PCIC protocol for streaming pixel data and triggered image acquisition."""
from __future__ import annotations
import ifm3dpy.framegrabber
import typing
import datetime
import ifm3dpy
import ifm3dpy.device

__all__ = [
    "Frame",
    "FrameGrabber",
    "buffer_id"
]


class Frame():
    """
    Represent a frame of data received from the the device.
    """
    def frame_count(self) -> int: 
        """
        Get the frame count according to algorithm output
        """
    def get_buffer(self, id: buffer_id) -> numpy.ndarray: 
        """
        Get the buffer with the given id
        """
    def get_buffers(self) -> typing.List[buffer_id]: 
        """
        Get the list of available buffers
        """
    def has_buffer(self, id: buffer_id) -> bool: 
        """
        Check if a buffer with the given id is available in this frame
        """
    def timestamps(self) -> typing.List[datetime.datetime]: 
        """
        Get the timestamps of the frame
        """
    pass
class FrameGrabber():
    """
    Implements a TCP FrameGrabber connected to a provided Camera
    """
    def __init__(self, cam: ifm3dpy.device.Device, pcic_port: typing.Optional[int] = None) -> None: 
        """
        Constructor

        Parameters
        ----------
        cam : ifm3dpy.Camera
            The camera instance to grab frames from.

        pcic_port : uint16
            The PCIC port
        """
    def is_masking(self) -> bool: 
        """
        Returns
        -------
        Masking flag
        """
    def is_running(self) -> bool: 
        """
        Returns true if the worker thread is currently running
        """
    def on_async_error(self, callback: typing.Callable[[int, str], None] = None) -> None: 
        """
        This function will enable the async error messages on device.
        The callback will be executed whenever a async error
        are avaliable. It receives a error code and error string
        to the received async error as an argument. 
        """
    def on_async_notification(self, callback: typing.Callable[[str, str], None] = None) -> None: 
        """
        This function will enable the async notifications on device.
        The callback will be executed whenever a async notification
        is avaliable. It receives a message id and payload string
        """
    def on_error(self, callback: typing.Callable[[ifm3dpy.device.Error], None] = None) -> None: 
        """
        on_error(self: ifm3dpy.framegrabber.FrameGrabber, callback: Callable[[ifm3dpy.device.Error], None] = None) -> None


        The callback will be executed whenever an error condition
        occur while grabbing the data from device.
        """
    def on_new_frame(self, callback: typing.Callable[[Frame], None] = None) -> None: 
        """
        The callback will be executed whenever a new frame is available.
        It receives the frame as an argument.
        """
    def set_masking(self, arg0: bool) -> None: 
        """
        Enable/Disable masking on supported buffers
        Note: ifm3dpy.buffer_id.CONFIDENCE_IMAGE should be in schema  list passed to ifm3dpy.FrameGrabber.Start method

        Parameters
        ----------
        mask
            flag to enable/disable masking.
        """
    def start(self, buffers: typing.Sequence[typing.Union[int, buffer_id]] = [], pcic_format: typing.Optional[dict] = None) -> ifm3dpy.Awaitable: 
        """
        start(self: ifm3dpy.framegrabber.FrameGrabber, buffers: typing.Sequence[Union[int, ifm3dpy.framegrabber.buffer_id]] = [], pcic_format: Optional[dict] = None) -> ifm3dpy.Awaitable


        Starts the worker thread for streaming in pixel data from the device

        Returns
        -------
        FutureAwaitable

            Resolves when framegrabber is ready to receive frames. 

        Parameters
        ----------
        buffers : List[uint64]
            A List of buffer_ids for receiving, passing in an List
            set will received all available images. The buffer_ids are specific to
            the current Organizer. See buffer_id for a list of buffer_ids available
            with the default Organizer

        pcicFormat : Dict
            allows to manually set a PCIC pcicFormat for
            asynchronous results. See ifm3d::make_schema for generation logic of the
            default pcicFormat. Manually setting the pcicFormat should rarely be needed and
            most usecases should be covered by the default generated pcicFormat.

            Note: The FrameGrabber is relying on some specific formatting rules, if
            they are missing from the pcicFormat the FrameGrabber will not be able to
            extract the image data.
        """
    def stop(self) -> ifm3dpy.Awaitable: 
        """
        Stops the worker thread for streaming in pixel data from the device

        Returns
        -------
        FutureAwaitable

            Resolves when framgrabber stops.
        """
    def sw_trigger(self) -> ifm3dpy.Awaitable: 
        """
        Triggers the device for image acquisition

        You should be sure to set the `TriggerMode` for your application to
        `SW` in order for this to be effective. This function
        simply does the triggering, data are still received asynchronously via
        `wait_for_frame()`.

        Calling this function when the device is not in `SW` trigger mode or on
        a device that does not support software-trigger should result in a NOOP
        and no error will be returned (no exceptions thrown). However, we do not
        recommend calling this function in a tight framegrabbing loop when you
        know it is not needed. The "cost" of the NOOP is undefined and incurring
        it is not recommended.
        """
    def wait_for_frame(self) -> ifm3dpy.FrameAwaitable: 
        """
        Returns an Awaitable that will resolve when a new frame is available
        """
    pass
class buffer_id():
    """
    Enum: buffer_id available for use with the default Organizer.

    Members:

      RADIAL_DISTANCE_IMAGE

      NORM_AMPLITUDE_IMAGE

      AMPLITUDE_IMAGE

      GRAYSCALE_IMAGE

      RADIAL_DISTANCE_NOISE

      REFLECTIVITY

      CARTESIAN_X_COMPONENT

      CARTESIAN_Y_COMPONENT

      CARTESIAN_Z_COMPONENT

      CARTESIAN_ALL

      UNIT_VECTOR_ALL

      MONOCHROM_2D_12BIT

      MONOCHROM_2D

      JPEG_IMAGE

      CONFIDENCE_IMAGE

      DIAGNOSTIC

      JSON_DIAGNOSTIC

      EXTRINSIC_CALIB

      INTRINSIC_CALIB

      INVERSE_INTRINSIC_CALIBRATION

      TOF_INFO

      O3R_DISTANCE_IMAGE_INFO

      RGB_INFO

      O3R_RGB_IMAGE_INFO

      JSON_MODEL

      ALGO_DEBUG

      O3R_ODS_OCCUPANCY_GRID

      O3R_ODS_INFO

      XYZ

      EXPOSURE_TIME

      ILLUMINATION_TEMP
    """
    def __eq__(self, other: object) -> bool: ...
    def __getstate__(self) -> int: ...
    def __hash__(self) -> int: ...
    def __index__(self) -> int: ...
    def __init__(self, value: int) -> None: ...
    def __int__(self) -> int: ...
    def __ne__(self, other: object) -> bool: ...
    def __repr__(self) -> str: ...
    def __setstate__(self, state: int) -> None: ...
    @property
    def name(self) -> str:
        """
        :type: str
        """
    @property
    def value(self) -> int:
        """
        :type: int
        """
    ALGO_DEBUG: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.ALGO_DEBUG: 900>
    AMPLITUDE_IMAGE: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.AMPLITUDE_IMAGE: 103>
    CARTESIAN_ALL: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.CARTESIAN_ALL: 203>
    CARTESIAN_X_COMPONENT: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.CARTESIAN_X_COMPONENT: 200>
    CARTESIAN_Y_COMPONENT: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.CARTESIAN_Y_COMPONENT: 201>
    CARTESIAN_Z_COMPONENT: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.CARTESIAN_Z_COMPONENT: 202>
    CONFIDENCE_IMAGE: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.CONFIDENCE_IMAGE: 300>
    DIAGNOSTIC: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.DIAGNOSTIC: 302>
    EXPOSURE_TIME: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.EXPOSURE_TIME: 4294967296>
    EXTRINSIC_CALIB: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.EXTRINSIC_CALIB: 400>
    GRAYSCALE_IMAGE: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.GRAYSCALE_IMAGE: 104>
    ILLUMINATION_TEMP: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.ILLUMINATION_TEMP: 4294967297>
    INTRINSIC_CALIB: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.INTRINSIC_CALIB: 401>
    INVERSE_INTRINSIC_CALIBRATION: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.INVERSE_INTRINSIC_CALIBRATION: 402>
    JPEG_IMAGE: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.JPEG_IMAGE: 260>
    JSON_DIAGNOSTIC: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.JSON_DIAGNOSTIC: 305>
    JSON_MODEL: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.JSON_MODEL: 500>
    MONOCHROM_2D: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.MONOCHROM_2D: 251>
    MONOCHROM_2D_12BIT: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.MONOCHROM_2D_12BIT: 250>
    NORM_AMPLITUDE_IMAGE: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.NORM_AMPLITUDE_IMAGE: 101>
    O3R_DISTANCE_IMAGE_INFO: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.TOF_INFO: 420>
    O3R_ODS_INFO: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.O3R_ODS_INFO: 1001>
    O3R_ODS_OCCUPANCY_GRID: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.O3R_ODS_OCCUPANCY_GRID: 1000>
    O3R_RGB_IMAGE_INFO: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.RGB_INFO: 421>
    RADIAL_DISTANCE_IMAGE: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.RADIAL_DISTANCE_IMAGE: 100>
    RADIAL_DISTANCE_NOISE: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.RADIAL_DISTANCE_NOISE: 105>
    REFLECTIVITY: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.REFLECTIVITY: 107>
    RGB_INFO: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.RGB_INFO: 421>
    TOF_INFO: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.TOF_INFO: 420>
    UNIT_VECTOR_ALL: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.UNIT_VECTOR_ALL: 223>
    XYZ: ifm3dpy.framegrabber.buffer_id # value = <buffer_id.XYZ: 4294967295>
    __members__: dict # value = {'RADIAL_DISTANCE_IMAGE': <buffer_id.RADIAL_DISTANCE_IMAGE: 100>, 'NORM_AMPLITUDE_IMAGE': <buffer_id.NORM_AMPLITUDE_IMAGE: 101>, 'AMPLITUDE_IMAGE': <buffer_id.AMPLITUDE_IMAGE: 103>, 'GRAYSCALE_IMAGE': <buffer_id.GRAYSCALE_IMAGE: 104>, 'RADIAL_DISTANCE_NOISE': <buffer_id.RADIAL_DISTANCE_NOISE: 105>, 'REFLECTIVITY': <buffer_id.REFLECTIVITY: 107>, 'CARTESIAN_X_COMPONENT': <buffer_id.CARTESIAN_X_COMPONENT: 200>, 'CARTESIAN_Y_COMPONENT': <buffer_id.CARTESIAN_Y_COMPONENT: 201>, 'CARTESIAN_Z_COMPONENT': <buffer_id.CARTESIAN_Z_COMPONENT: 202>, 'CARTESIAN_ALL': <buffer_id.CARTESIAN_ALL: 203>, 'UNIT_VECTOR_ALL': <buffer_id.UNIT_VECTOR_ALL: 223>, 'MONOCHROM_2D_12BIT': <buffer_id.MONOCHROM_2D_12BIT: 250>, 'MONOCHROM_2D': <buffer_id.MONOCHROM_2D: 251>, 'JPEG_IMAGE': <buffer_id.JPEG_IMAGE: 260>, 'CONFIDENCE_IMAGE': <buffer_id.CONFIDENCE_IMAGE: 300>, 'DIAGNOSTIC': <buffer_id.DIAGNOSTIC: 302>, 'JSON_DIAGNOSTIC': <buffer_id.JSON_DIAGNOSTIC: 305>, 'EXTRINSIC_CALIB': <buffer_id.EXTRINSIC_CALIB: 400>, 'INTRINSIC_CALIB': <buffer_id.INTRINSIC_CALIB: 401>, 'INVERSE_INTRINSIC_CALIBRATION': <buffer_id.INVERSE_INTRINSIC_CALIBRATION: 402>, 'TOF_INFO': <buffer_id.TOF_INFO: 420>, 'O3R_DISTANCE_IMAGE_INFO': <buffer_id.TOF_INFO: 420>, 'RGB_INFO': <buffer_id.RGB_INFO: 421>, 'O3R_RGB_IMAGE_INFO': <buffer_id.RGB_INFO: 421>, 'JSON_MODEL': <buffer_id.JSON_MODEL: 500>, 'ALGO_DEBUG': <buffer_id.ALGO_DEBUG: 900>, 'O3R_ODS_OCCUPANCY_GRID': <buffer_id.O3R_ODS_OCCUPANCY_GRID: 1000>, 'O3R_ODS_INFO': <buffer_id.O3R_ODS_INFO: 1001>, 'XYZ': <buffer_id.XYZ: 4294967295>, 'EXPOSURE_TIME': <buffer_id.EXPOSURE_TIME: 4294967296>, 'ILLUMINATION_TEMP': <buffer_id.ILLUMINATION_TEMP: 4294967297>}
    pass
