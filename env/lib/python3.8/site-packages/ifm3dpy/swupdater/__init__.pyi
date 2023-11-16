"""Provides utilities for managing the SWUpdate subsystem of the camera."""
from __future__ import annotations
import ifm3dpy.swupdater
import typing
import ifm3dpy.device

__all__ = [
    "SWUpdater"
]


class SWUpdater():
    """
    Class for managing an instance of an swupdater
    """
    def __init__(self, cam: ifm3dpy.device.Device, cb: typing.Optional[typing.Callable[[float, str], None]] = None, swupdate_recovery_port: int = 8080) -> None: 
        """
        Constructor

        Parameters
        ----------
        cam : ifm3dpy.Camera
            The camera instance to grab frames from

        cb : callback function  with parameters (float, string)
            A  function for progress of the update

        swupdate_recovery_port : uint16_t
            The Swupdate communication port
        """
    def flash_firmware(self, swu_file: str, timeout_millis: int = 0) -> bool: 
        """
        Flash the firmware .swu file to the device

        Parameter
        ---------

        swu_file : string
           A .swu file to flash on the device

        timeout_millis : long
           A timeout value in milliseconds

        Returns
        -------
        bool
            True if firmware upload is successful
        """
    def reboot_to_productive(self) -> None: 
        """
        Reboot the device in productive mode
        """
    def reboot_to_recovery(self) -> None: 
        """
        Reboot the device in Recovery Modes
        """
    def wait_for_productive(self, timeout_millis: int = 0) -> bool: 
        """
        Waits and check for device in productive mode
        till timeout

        Parameter
        ---------

        timeout_millis : long
            A timeout value in milliseconds

        Returns
        -------
        bool
            True if device in the productive mode
        """
    def wait_for_recovery(self, timeout_millis: int = 0) -> bool: 
        """
        Waits and check for device in recovery mode
        till timeout

        Parameter
        ---------

        timeout_millis : long
            A timeout value in milliseconds

        Returns
        -------
        bool
            True if device in the recovery mode
        """
    pass
