"""Provides an implementation of the XMLRPC protocol for configuring the camera and pmd imager settings."""
from __future__ import annotations
import ifm3dpy.device
import typing

__all__ = [
    "Device",
    "Error",
    "LegacyDevice",
    "O3R",
    "PortInfo",
    "SemVer"
]


class Device():
    """
    Base class for managing an instance of an all cameras
    """
    class boot_mode():
        """
        Enum: Camera boot up modes.

        Members:

          PRODUCTIVE : the normal runtime firmware comes up

          RECOVERY : allows you to flash new firmware
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
        PRODUCTIVE: ifm3dpy.device.Device.boot_mode # value = <boot_mode.PRODUCTIVE: 0>
        RECOVERY: ifm3dpy.device.Device.boot_mode # value = <boot_mode.RECOVERY: 1>
        __members__: dict # value = {'PRODUCTIVE': <boot_mode.PRODUCTIVE: 0>, 'RECOVERY': <boot_mode.RECOVERY: 1>}
        pass
    class device_family():
        """
        Enum: The family of the device

        Members:

          UNKNOWN

          O3D

          O3X

          O3R
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
        O3D: ifm3dpy.device.Device.device_family # value = <device_family.O3D: 1>
        O3R: ifm3dpy.device.Device.device_family # value = <device_family.O3R: 3>
        O3X: ifm3dpy.device.Device.device_family # value = <device_family.O3X: 2>
        UNKNOWN: ifm3dpy.device.Device.device_family # value = <device_family.UNKNOWN: 0>
        __members__: dict # value = {'UNKNOWN': <device_family.UNKNOWN: 0>, 'O3D': <device_family.O3D: 1>, 'O3X': <device_family.O3X: 2>, 'O3R': <device_family.O3R: 3>}
        pass
    class import_flags():
        """
        Enum: Import flags used when importing a Vision Assistant configuration

        Members:

          GLOBAL

          NET

          APPS
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
        APPS: ifm3dpy.device.Device.import_flags # value = <import_flags.APPS: 16>
        GLOBAL: ifm3dpy.device.Device.import_flags # value = <import_flags.GLOBAL: 1>
        NET: ifm3dpy.device.Device.import_flags # value = <import_flags.NET: 2>
        __members__: dict # value = {'GLOBAL': <import_flags.GLOBAL: 1>, 'NET': <import_flags.NET: 2>, 'APPS': <import_flags.APPS: 16>}
        pass
    class mfilt_mask_size():
        """
        Enum: Convenient constants for median filter mask sizes

        Members:

          _3x3

          _5x5
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
        _3x3: ifm3dpy.device.Device.mfilt_mask_size # value = <mfilt_mask_size._3x3: 0>
        _5x5: ifm3dpy.device.Device.mfilt_mask_size # value = <mfilt_mask_size._5x5: 1>
        __members__: dict # value = {'_3x3': <mfilt_mask_size._3x3: 0>, '_5x5': <mfilt_mask_size._5x5: 1>}
        pass
    class operating_mode():
        """
        Enum: Camera operating modes

        Members:

          RUN : streaming pixel data

          EDIT : configuring the device/applications
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
        EDIT: ifm3dpy.device.Device.operating_mode # value = <operating_mode.EDIT: 1>
        RUN: ifm3dpy.device.Device.operating_mode # value = <operating_mode.RUN: 0>
        __members__: dict # value = {'RUN': <operating_mode.RUN: 0>, 'EDIT': <operating_mode.EDIT: 1>}
        pass
    class spatial_filter():
        """
        Enum: Convenience constants for spatial filter types

        Members:

          OFF

          MEDIAN

          MEAN

          BILATERAL
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
        BILATERAL: ifm3dpy.device.Device.spatial_filter # value = <spatial_filter.BILATERAL: 3>
        MEAN: ifm3dpy.device.Device.spatial_filter # value = <spatial_filter.MEAN: 2>
        MEDIAN: ifm3dpy.device.Device.spatial_filter # value = <spatial_filter.MEDIAN: 1>
        OFF: ifm3dpy.device.Device.spatial_filter # value = <spatial_filter.OFF: 0>
        __members__: dict # value = {'OFF': <spatial_filter.OFF: 0>, 'MEDIAN': <spatial_filter.MEDIAN: 1>, 'MEAN': <spatial_filter.MEAN: 2>, 'BILATERAL': <spatial_filter.BILATERAL: 3>}
        pass
    class temporal_filter():
        """
        Enum: Convenience constants for temporal filter types

        Members:

          OFF

          MEAN

          ADAPTIVE_EXP
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
        ADAPTIVE_EXP: ifm3dpy.device.Device.temporal_filter # value = <temporal_filter.ADAPTIVE_EXP: 2>
        MEAN: ifm3dpy.device.Device.temporal_filter # value = <temporal_filter.MEAN: 1>
        OFF: ifm3dpy.device.Device.temporal_filter # value = <temporal_filter.OFF: 0>
        __members__: dict # value = {'OFF': <temporal_filter.OFF: 0>, 'MEAN': <temporal_filter.MEAN: 1>, 'ADAPTIVE_EXP': <temporal_filter.ADAPTIVE_EXP: 2>}
        pass
    class trigger_mode():
        """
        Enum: Image acquisition trigger modes

        Members:

          FREE_RUN

          SW
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
        FREE_RUN: ifm3dpy.device.Device.trigger_mode # value = <trigger_mode.FREE_RUN: 1>
        SW: ifm3dpy.device.Device.trigger_mode # value = <trigger_mode.SW: 2>
        __members__: dict # value = {'FREE_RUN': <trigger_mode.FREE_RUN: 1>, 'SW': <trigger_mode.SW: 2>}
        pass
    def __init__(self, ip: str = '192.168.0.69', xmlrpc_port: int = 80, password: str = '', throw_if_unavailable: bool = True) -> None: 
        """
        Constructor

        Parameters
        ----------
        ip : string, optional
            The ip address of the camera. Defaults to 192.168.0.69.

        xmlrpc_port : uint, optional
            The tcp port the sensor's XMLRPC server is listening on. Defaults to
            port 80.

        password : string, optional
            Password required for establishing an "edit session" with the sensor.
            Edit sessions allow for mutating camera parameters and persisting
            those changes. Defaults to '' (no password).
        """
    def am_i(self, family: Device.device_family) -> bool: 
        """
        Checking whether a device is one of the specified device family

        Parameters
        ----------
        family : CameraBase.device_family
            The family to check for

        Returns
        -------
        bool
            True if the device is of the specified family
        """
    def check_minimum_firmware_version(self, major: int, minor: int, patch: int) -> bool: 
        """
        Checks for a minimum ifm camera software version

        Parameters
        ----------
        major : int
            Major version of software

        minor : int
            Minor Version of software

        patch : int
            Patch Number of software

        Returns
        -------
        bool
            True if current software version is greater
            or equal to the value passed
        """
    def device_parameter(self, key: str) -> str: 
        """
        Convenience accessor for extracting a device parameter

        No edit session is created on the camera

        Parameters
        ----------
        key : str
            Name of the parameter to extract

        Returns
        -------
        str
            Value of the requested parameter

        Raises
        ------
        RuntimeError
        """
    def device_type(self, use_cached: bool = True) -> str: 
        """
        Obtains the device type of the connected camera.

        This is a convenience function for extracting out the device type of the
        connected camera. The primary intention of this function is for internal
        usage (i.e., to trigger conditional logic based on the model hardware
        we are talking to) however, it will likely be useful in
        application-level logic as well, so, it is available in the public
        interface.

        Parameters
        ----------
        use_cached : bool
            If set to true, a cached lookup of the device type will be used as
            the return value. If false, it will make a network call to the camera
            to get the "real" device type. The only reason for setting this to
            `false` would be if you expect over the lifetime of your camera
            instance that you will swap out (for example) an O3D for an O3X (or
            vice versa) -- literally, swapping out the network cables while an
            object instance is still alive. If that is not something you are
            worried about, leaving this set to true should result in a signficant
            performance increase.

        Returns
        -------
        str
            Type of device connected
        """
    def firmware_version(self) -> SemVer: 
        """
        Version of firmware installed on device

        Returns
        -------
        SemVer
            Firmware version 
        """
    def force_trigger(self) -> None: 
        """
        Sends a S/W trigger to the camera over XMLRPC.

        The O3X does not S/W trigger over PCIC, so, this function
        has been developed specficially for it. For the O3D, this is a NOOP.
        """
    def from_json(self, json: dict) -> None: 
        """
        Configures the camera based on the parameter values of the passed in
        JSON. This function is _the_ way to tune the
        camera/application/imager/etc. parameters.

        Parameters
        ----------
        json : dict
            A json object encoding a camera configuration to apply
            to the hardware.

        Raises
        ------
        RuntimeError
            If this raises an exception, you are
            encouraged to check the log file as a best effort is made to be
            as descriptive as possible as to the specific error that has
            occured.
        """
    def reboot(self, mode: Device.boot_mode = boot_mode.PRODUCTIVE) -> None: 
        """
        Reboot the sensor

        Parameters
        ----------
        mode : CameraBase.boot_mode
            The system mode to boot into upon restart of the sensor

        Raises
        ------
        RuntimeError
        """
    def to_json(self) -> dict: 
        """
        A JSON object containing the state of the camera

        Returns
        -------
        dict
            Camera JSON, compatible with python's json module

        Raises
        ------
        RuntimeError
        """
    def trace_logs(self, count: int) -> typing.List[str]: 
        """
        Delivers the trace log from the camera

        A session is not required to call this function.

        Parameters
        ----------
        count : int
            Number of entries to retrieve

        Returns
        -------
        list[str]
            List of strings for each entry in the tracelog
        """
    def who_am_i(self) -> Device.device_family: 
        """
        Retrieve the device family of the connected device

        Returns
        -------
        CameraBase.device_family
            The device family
        """
    @property
    def ip(self) -> str:
        """
        The IP address associated with this Camera instance

        :type: str
        """
    @property
    def xmlrpc_port(self) -> int:
        """
        The XMLRPC port associated with this Camera instance

        :type: int
        """
    pass
class Error(RuntimeError, Exception, BaseException):
    """
    Exception wrapper for library and system errors encountered by the library.
    """
    @property
    def code(self) -> None:
        """
        Error Code

        :type: None
        """
    @property
    def message(self) -> None:
        """
        Exception message

        :type: None
        """
    @property
    def what(self) -> None:
        """
        String representation of the error including the error code and optionally the message

        :type: None
        """
    pass
class LegacyDevice(Device):
    """
    Class for managing an instance of an O3D/O3X Camera
    """
    def __init__(self, ip: str = '192.168.0.69', xmlrpc_port: int = 80, password: str = '') -> None: 
        """
        Constructor

        Parameters
        ----------
        ip : string, optional
            The ip address of the camera. Defaults to 192.168.0.69.

        xmlrpc_port : uint, optional
            The tcp port the sensor's XMLRPC server is listening on. Defaults to
            port 80.

        password : string, optional
            Password required for establishing an "edit session" with the sensor.
            Edit sessions allow for mutating camera parameters and persisting
            those changes. Defaults to '' (no password).
        """
    def active_application(self) -> int: 
        """
        Returns the index of the active application.

        A negative number indicates no application is marked as active on the
        sensor.
        """
    def application_list(self) -> dict: 
        """
        Delivers basic information about all applications stored on the device.
        A call to this function does not require establishing a session with the
        camera.

        The returned information is encoded as an array of JSON objects.
        Each object in the array is basically a dictionary with the following
        keys: 'index', 'id', 'name', 'description', 'active'

        Returns
        -------
        dict
            A JSON encoding of the application information

        Raises
        ------
        RuntimeError
        """
    def application_types(self) -> typing.List[str]: 
        """
        Lists the valid application types supported by the sensor.

        Returns
        -------
        list[str]
            List of strings of the available types of applications supported by
            the sensor. Each element in the list is a string suitable to passing
            to 'CreateApplication'.

        Raises
        ------
        RuntimeError
        """
    @typing.overload
    def cancel_session(self) -> bool: 
        """
        Explictly stops the current session with the sensor.

        Returns
        -------
        bool
            Indicates success or failure. On failure, check the ifm3d system log
            for details.



        Attempts to cancel a session with a particular session id.

        Parameters
        ----------
        sid : str
            Session ID to cancel.

        Returns
        -------
        bool
            Indicates success or failure. On failure, check the ifm3d system log
            for details.
        """
    @typing.overload
    def cancel_session(self, sid: str) -> bool: ...
    def copy_application(self, idx: int) -> int: 
        """
        Creates a new application by copying the configuration of another
        application. The device will generate an ID for the new application and
        put it on a free index.

        Parameters
        ----------
        idx : int
            The index of the application to copy

        Returns
        -------
        int
            Index of the new application

        Raises
        ------
        RuntimeError
        """
    def create_application(self, type: str = 'Camera') -> int: 
        """
        Creates a new application on the camera of the given type.

        To figure out valid `type`s, you should call the
        AvailableApplicationTypes()` method.

        Upon creation of the application, the embedded device will initialize
        all parameters as necessary based on the type. However, based on the
        type, the application may not be in an _activatable_ state. That is, it
        can be created and saved on the device, but it cannot be marked as
        active.

        Parameters
        ----------
        type : str, optional
            The (optional) application type to create. By default,
            it will create a new "Camera" application.

        Returns
        -------
        int
            The index of the new application.
        """
    def delete_application(self, idx: int) -> None: 
        """
        Deletes the application at the specified index from the sensor.

        Parameters
        ----------
        idx : int
            The index of the application to delete

        Raises
        ------
        RuntimeError
        """
    def export_ifm_app(self, idx: int) -> typing.List[int]: 
        """
        Export the application at the specified index into a byte array suitable
        for writing to a file. The exported bytes represent the ifm serialization
        of an application.

        This function provides compatibility with tools like IFM's Vision
        Assistant.

        Parameters
        ----------
        idx : int
            The index of the application to export.

        Returns
        -------
        list[int]
            A list of bytes representing the IFM serialization of the
            exported application.

        Raises
        ------
        RuntimeError
        """
    def export_ifm_config(self) -> typing.List[int]: 
        """
        Exports the entire camera configuration in a format compatible with
        Vision Assistant.

        Returns
        -------
        list[int]
        """
    def factory_reset(self) -> None: 
        """
        Sets the camera configuration back to the state in which it shipped from
        the ifm factory.
        """
    def heartbeat(self, hb: int) -> int: 
        """
        Sends a heartbeat message and sets the next heartbeat interval

        Heartbeat messages are used to keep a session with the sensor
        alive. This function sends a heartbeat message to the sensor and sets
        when the next heartbeat message is required.

        Parameters
        ----------
        hb : int
            The time (seconds) of when the next heartbeat message will
            be required.

        Returns
        -------
        int
            The current timeout interval in seconds for heartbeat messages

        Raises
        ------
        RuntimeError
        """
    def imager_types(self) -> typing.List[str]: 
        """
        Lists the valid imager types supported by the sensor.

        Returns
        -------
        list[str]
            List of strings of the available imager types supported by the sensor

        Raises
        ------
        RuntimeError
        """
    def import_ifm_app(self, bytes: typing.List[int]) -> int: 
        """
        Import the IFM-encoded application.

        This function provides compatibility with tools like IFM's Vision
        Assistant. An application configuration exported from VA, can be
        imported using this function.

        Parameters
        ----------
        bytes : list[int]
            The raw bytes from the zip'd JSON file. NOTE: This
            function will base64 encode the data for tranmission
            over XML-RPC.

        Returns
        -------
        int
            The index of the imported application.
        """
    def import_ifm_config(self, bytes: typing.List[int], flags: int = 0) -> None: 
        """
        Imports the entire camera configuration in a format compatible with
        Vision Assistant.

        Parameters
        ----------
        bytes : list[int]
            The camera configuration, serialized in the ifm format

        flags : int
        """
    def request_session(self) -> str: 
        """
        Requests an edit-mode session with the camera.

        In order to (permanently) mutate parameters on the camera, an edit
        session needs to be established. Only a single edit sesson may be
        established at any one time with the camera (think of it as a global
        mutex on the camera state -- except if you ask for the mutex and it is
        already taken, an exception will be thrown).

        Most typical use-cases for end-users will not involve establishing an
        edit-session with the camera. To mutate camera parameters, the
        `FromJSON` family of functions should be used, which, under-the-hood,
        on the user's behalf, will establish the edit session and gracefully
        close it. There is an exception. For users who plan to modulate imager
        parameters (temporary parameters) on the fly while running the
        framegrabber, managing the session manually is necessary. For this
        reason, we expose this method in the public `Camera` interface.

        NOTE: The session timeout is implicitly set to `ifm3d::MAX_HEARTBEAT`
        after the session has been successfully established.

        Returns
        -------
        str
            The session id issued or accepted by the camera (see
            IFM3D_SESSION_ID environment variable)

        Raises
        ------
        RuntimeError

        @throws ifm3d::error_t if an error is encountered.
        """
    def set_current_time(self, epoch_secs: int = -1) -> None: 
        """
        Sets the current time on the camera

        Parameters
        ----------
        epoch_secs : int, optional
            Time since the Unix epoch in seconds. A value less than 0 will
            implicity set the time to the current system time.
        """
    def set_temporary_application_parameters(self, params: typing.Dict[str, str]) -> None: 
        """
        Sets temporary application parameters in run mode.

        The changes are not persistent and are lost when entering edit mode or
        turning the device off. The parameters "ExposureTime" and
        "ExposureTimeRatio" of the imager configuration are supported. All
        additional parameters are ignored (for now). Exposure times are clamped
        to their allowed range, depending on the exposure mode. The user must
        provide the complete set of parameters depending on the exposure mode,
        i.e., "ExposureTime" only for single exposure modes and both
        "ExposureTime" and "ExposureTimeRatio" for double exposure
        modes. Otherwise, behavior is undefined.

        Parameters
        ----------
        params : dict[str, str]
            The parameters to set on the camera.

        Raises
        ------
        RuntimeError
        """
    def unit_vectors(self) -> typing.List[int]: 
        """
        For cameras that support fetching the Unit Vectors over XML-RPC, this
        function will return those data as a binary blob.

        Returns
        -------
        list[int]
        """
    @property
    def password(self) -> str:
        """
        The password associated with this Camera instance

        :type: str
        """
    @password.setter
    def password(self, arg1: str) -> None:
        """
        The password associated with this Camera instance
        """
    @property
    def session_id(self) -> str:
        """
        Retrieves the active session ID

        :type: str
        """
    pass
class O3R(Device):
    """
    Class for managing an instance of an O3R Camera
    """
    def __init__(self, ip: str = '192.168.0.69', xmlrpc_port: int = 80) -> None: 
        """
        Constructor

        Parameters
        ----------
        ip : string, optional
            The ip address of the camera. Defaults to 192.168.0.69.

        xmlrpc_port : uint, optional
            The tcp port the sensor's XMLRPC server is listening on. Defaults to
            port 80.

        password : string, optional
            Password required for establishing an "edit session" with the sensor.
            Edit sessions allow for mutating camera parameters and persisting
            those changes. Defaults to '' (no password).
        """
    def factory_reset(self, keep_network_settings: bool) -> None: 
        """
        Sets the camera configuration back to the state in which it shipped from
        the ifm factory.

        Parameters
        ----------
        keep_network_settings : bool
            A bool indicating wether to keep the current network settings
        """
    def get(self, path: typing.List[str] = []) -> dict: 
        """
        Returns the configuration formatted as JSON based on a path.
        If the path is empty, returns the whole configuration.

        Returns
        -------
        dict
            The JSON configuration for the list of object path fragments
        """
    def get_diagnostic(self) -> dict: 
        """
        Returns the content of the diagnostic memory formatted in JSON

        Returns
        -------
        dict
        """
    def get_diagnostic_filter_schema(self) -> dict: 
        """
        Returns the JSON schema for the filter expression provided to the 
        getFiltered() method

        Returns
        -------
        dict
            The JSON schema
        """
    def get_diagnostic_filtered(self, filter: dict) -> dict: 
        """
        Returns the content of the diagnostic memory formatted in JSON
        and filtered according to the JSON filter expression

        Parameters
        ----------
        filter : dict
            A filter expression in JSON format

        Returns
        -------
        dict 
        """
    def get_init(self) -> dict: 
        """
        Return the initial JSON configuration.

        Returns
        -------
        dict
            The initial JSON configuration
        """
    def get_init_status(self) -> str: 
        """
        Returns the init status of the device

        Returns
        -------
        dict
            The init status of the device
        """
    def get_schema(self) -> dict: 
        """
        Returns the current JSON schema configuration

        Returns
        -------
        dict
            The current json schema configuration
        """
    def lock(self, password: str) -> None: 
        """
        Release the lock from the Device

        Returns
        -------
        string
            The password used to unlock the device
        """
    def port(self, port: str) -> PortInfo: 
        """
        Returns information about a given physical port

        Parameters
        ----------
        port : str
            the port for which to get the information

        Returns
        -------
        PortInfo
            the port information
        """
    def ports(self) -> typing.List[PortInfo]: 
        """
        Returns a list containing information about all connected physical ports

        Returns
        -------
        List[PortInfo]
            the list of Ports
        """
    def reboot_to_recovery(self) -> None: 
        """
        Reboot the device into Recovery Mode

        Raises
        ------
        RuntimeError
        """
    def remove(self, json_pointer: str) -> None: 
        """
        Removes an object from the JSON. The scope of this method is limited to
        the following regular expressions

         * ^\/applications\/instances\/app\d+$
         * ^\/device\/log\/components\/[a-zA-Z0-9\-_]+$

        Parameters
        ----------
        json_pointer : string
            A JSON Pointer to the object to be removed.
        """
    def reset(self, json_pointer: str) -> None: 
        """
        Sets the default value of an object inside the JSON. The object is
        addressed by a JSON Pointer. The object is resetted to the values
        defined in the JSON schema.

        Parameters
        ----------
        json_pointer : string
            A JSON Pointer to the object to be set to default.
        """
    def resolve_config(self, json_pointer: str) -> dict: 
        """
        * Returns a part of the configuration formatted as JSON based on a
        * JSON pointer.

         Returns
         -------
         dict
             The partial JSON configuration for the given JSON pointer
        """
    def save_init(self) -> None: 
        """
        Save to current temporary JSON configuration as initial JSON
        configuration
        """
    def set(self, json: dict) -> None: 
        """
        Overwrites parts of the temporary JSON configuration which is achieved
        by merging the provided JSON fragment with the current temporary JSON.

        Parameters
        ----------
        json : dict
            The new temporay JSON configuration of the device.
        """
    def unlock(self, password: str) -> None: 
        """
        Locks the device until it is unlocked.
        If the device is unlocked and an empty password is provided the password
        protection is removed.

        Returns
        -------
        string
            The password used to lock the device
        """
    pass
class PortInfo():
    """
    Provides information about a connected Port
    """
    def __repr__(self) -> str: ...
    @property
    def pcic_port(self) -> int:
        """
        The assigned pcic port.

        :type: int
        """
    @property
    def port(self) -> str:
        """
        The name of the port.

        :type: str
        """
    @property
    def type(self) -> str:
        """
        The type of the conntected sensor.

        :type: str
        """
    pass
class SemVer():
    """
    struct for holding the version information
    """
    @staticmethod
    def Parse(arg0: str) -> SemVer: ...
    def __eq__(self, arg0: SemVer) -> bool: ...
    def __ge__(self, arg0: SemVer) -> bool: ...
    def __gt__(self, arg0: SemVer) -> bool: ...
    def __init__(self, major: int, minor: int, build: int, prerelease: typing.Optional[str] = None, build_meta: typing.Optional[str] = None) -> None: ...
    def __le__(self, arg0: SemVer) -> bool: ...
    def __lt__(self, arg0: SemVer) -> bool: ...
    def __ne__(self, arg0: SemVer) -> bool: ...
    def __repr__(self) -> str: ...
    __hash__ = None
    pass
