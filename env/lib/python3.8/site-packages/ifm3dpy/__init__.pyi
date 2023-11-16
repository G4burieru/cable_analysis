"""
    Bindings for the ifm3d Library

    **Variables**

    .. csv-table::

       "__version__", "The ifm3d version."
     "__package__", "The ifm3d package."
     "DEFAULT_IP", "The default IP to connect to."
     "DEFAULT_XMLRPC_PORT", "The default XMLRPC port."
     "DEFAULT_PASSWORD", "The default password."
     "O3D_TIME_SUPPORT_MAJOR", "Constant for querying for O3D time support."
     "O3D_TIME_SUPPORT_MINOR", "Constant for querying for O3D time support."
     "O3D_TIME_SUPPORT_PATCH", "Constant for querying for O3D time support."
     "O3D_TMP_PARAMS_SUPPORT_MAJOR", "Constant for querying for O3D temporary parameter support."
     "O3D_TMP_PARAMS_SUPPORT_MINOR", "Constant for querying for O3D temporary parameter support."
     "O3D_TMP_PARAMS_SUPPORT_PATCH", "Constant for querying for O3D temporary parameter support."
     "O3D_INTRINSIC_PARAM_SUPPORT_MAJOR", "Constant for querying for O3D intrinsic parameter support."
     "O3D_INTRINSIC_PARAM_SUPPORT_MINOR", "Constant for querying for O3D intrinsic parameter support."
     "O3D_INTRINSIC_PARAM_SUPPORT_PATCH", "Constant for querying for O3D intrinsic parameter support."
     "O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MAJOR", "Constant for querying for O3D inverse intrinsic parameter support."
     "O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MINOR", "Constant for querying for O3D inverse intrinsic parameter support."
     "O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_PATCH", "Constant for querying for O3D inverse intrinsic parameter support."
"""
from __future__ import annotations
import ifm3dpy
import typing
from ifm3dpy.device import Device
from ifm3dpy.device import Error
from ifm3dpy.framegrabber import Frame
from ifm3dpy.framegrabber import FrameGrabber
from ifm3dpy.device import LegacyDevice
from ifm3dpy.device import O3R
from ifm3dpy.swupdater import SWUpdater
from ifm3dpy.device import SemVer
from ifm3dpy.framegrabber import buffer_id

__all__ = [
    "Awaitable",
    "DEFAULT_IP",
    "DEFAULT_PASSWORD",
    "DEFAULT_XMLRPC_PORT",
    "Device",
    "Error",
    "Frame",
    "FrameAwaitable",
    "FrameGrabber",
    "LegacyDevice",
    "O3D_INTRINSIC_PARAM_SUPPORT_MAJOR",
    "O3D_INTRINSIC_PARAM_SUPPORT_MINOR",
    "O3D_INTRINSIC_PARAM_SUPPORT_PATCH",
    "O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MAJOR",
    "O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MINOR",
    "O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_PATCH",
    "O3D_TIME_SUPPORT_MAJOR",
    "O3D_TIME_SUPPORT_MINOR",
    "O3D_TIME_SUPPORT_PATCH",
    "O3D_TMP_PARAMS_SUPPORT_MAJOR",
    "O3D_TMP_PARAMS_SUPPORT_MINOR",
    "O3D_TMP_PARAMS_SUPPORT_PATCH",
    "O3R",
    "SWUpdater",
    "SemVer",
    "buffer_id",
    "deserialize",
    "device",
    "framegrabber",
    "run",
    "swupdater"
]


class Awaitable():
    """
    Provides a mechanism to wait for completion of a task
    """
    def __await__(self) -> typing.Generator[None,None,None]: ...
    def __init__(self) -> None: 
        """
        Provides a mechanism to wait for completion of a task
        """
    def __iter__(self) -> typing.Generator[None,None,None]: ...
    def __next__(self) -> None: ...
    def wait(self) -> None: 
        """
        wait(self) -> None


        Blocks until the result becomes available.
        """
    def wait_for(self, timeout_ms: int) -> typing.Tuple[bool, None]: 
        """
        wait_for(self, timeout_ms: int) -> Tuple[bool, None]


        Blocks until specified timeout runs out or the result to becomes available. 

        :return: a tuple (True, Result) if a result was received within the timeout, (False, None) otherwise.
        """
    pass
class FrameAwaitable():
    """
    Provides a mechanism to access the frame object
    """
    def __await__(self) -> typing.Generator[Frame,Frame,Frame]: ...
    def __init__(self) -> None: 
        """
        Provides a mechanism to access the frame object
        """
    def __iter__(self) -> typing.Generator[Frame,Frame,Frame]: ...
    def __next__(self) -> None: ...
    def wait(self) -> Frame: 
        """
        wait(self) -> Frame


        Blocks until the result becomes available.
        """
    def wait_for(self, timeout_ms: int) -> typing.Tuple[bool, Frame]: 
        """
        wait_for(self, timeout_ms: int) -> Tuple[bool, Frame]


        Blocks until specified timeout runs out or the result to becomes available. 

        :return: a tuple (True, Result) if a result was received within the timeout, (False, None) otherwise.
        """
    pass
def _run_cmdtool() -> None:
    """
    Entry point for the ifm3dpy console application
    """
def run(arg0: list) -> typing.Tuple[int, str]:
    """
    This function provides python application interface to run command line tool

    Note: It is not recommended to use this for scripting as there is no way to 
    monitor progress or handle errors, instead please directly use the functions 
    provided by the corresponding modules. 
    This function is mainly intended to be used to integrate the ifm3d CLI into 
    existing console based applications.

    Parameters
    ----------
    argv : py::list
        command line parameter in the list. e.g. to call a 'ls' command
        ['ls', '--ip=192.168.0.69']

    Returns
    -------
    Tuple(int,string): py::tuple(int,string)
        execution state and output string.
    """
DEFAULT_IP = '192.168.0.69'
DEFAULT_PASSWORD = ''
DEFAULT_XMLRPC_PORT = 80
O3D_INTRINSIC_PARAM_SUPPORT_MAJOR = 1
O3D_INTRINSIC_PARAM_SUPPORT_MINOR = 23
O3D_INTRINSIC_PARAM_SUPPORT_PATCH = 0
O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MAJOR = 1
O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_MINOR = 30
O3D_INVERSE_INTRINSIC_PARAM_SUPPORT_PATCH = 4123
O3D_TIME_SUPPORT_MAJOR = 1
O3D_TIME_SUPPORT_MINOR = 20
O3D_TIME_SUPPORT_PATCH = 790
O3D_TMP_PARAMS_SUPPORT_MAJOR = 1
O3D_TMP_PARAMS_SUPPORT_MINOR = 20
O3D_TMP_PARAMS_SUPPORT_PATCH = 0
__version__ = '1.3.3'
