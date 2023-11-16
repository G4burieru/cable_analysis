"""Provides definitions and functions for deserializing structs sent over PCIC"""
from __future__ import annotations
import ifm3dpy.deserialize
import typing

__all__ = [
    "Calibration",
    "ExtrinsicOpticToUser",
    "ODSInfoV1",
    "ODSOccupancyGridV1",
    "RGBInfoV1",
    "TOFInfoV3",
    "TOFInfoV4"
]


class Calibration():
    """
    Class for managing an instance calibration parameters
    """
    def __init__(self) -> None: 
        """
        Constructor
        """
    @property
    def model_id(self) -> int:
        """
                Model Id for calibration parameters
              

        :type: int
        """
    @property
    def parameters(self) -> typing.List[float[32]]:
        """
                Parameters for calibration
              

        :type: typing.List[float[32]]
        """
    pass
class ExtrinsicOpticToUser():
    """
    Class for managing an instance Extrinsic parameters
    """
    def __init__(self) -> None: 
        """
        Constructor
        """
    @property
    def rot_x(self) -> float:
        """
                Extrinsic Parameter rotX
              

        :type: float
        """
    @property
    def rot_y(self) -> float:
        """
                Extrinsic Parameter rotY
              

        :type: float
        """
    @property
    def rot_z(self) -> float:
        """
                Extrinsic Parameter rotZ
              

        :type: float
        """
    @property
    def trans_x(self) -> float:
        """
                Extrinsic Parameter transX
              

        :type: float
        """
    @property
    def trans_y(self) -> float:
        """
                Extrinsic Parameter transY
              

        :type: float
        """
    @property
    def trans_z(self) -> float:
        """
                Extrinsic Parameter transZ
              

        :type: float
        """
    pass
class ODSInfoV1():
    """
    Class for managing an instance of an struct ODSInfoV1
    """
    def __init__(self) -> None: 
        """
        Constructor
        """
    @staticmethod
    def deserialize(arg0: numpy.ndarray[numpy.uint8]) -> ODSInfoV1: 
        """
        Deserialize ODS_INFO Buffer to ODSInfoV1 struct
        """
    @property
    def timestamp_ns(self) -> int:
        """
                Timestamp of zone information [ns]
              

        :type: int
        """
    @property
    def zone_config_id(self) -> int:
        """
                 User specific id to identify the zone configuration
              

        :type: int
        """
    @property
    def zone_occupied(self) -> typing.List[int[3]]:
        """
                 Array with three value of uint8_t
                 0 : zone is free 
                 1 : zone is occupied
              

        :type: typing.List[int[3]]
        """
    pass
class ODSOccupancyGridV1():
    """
    Class for managing an instance of an struct ODSOccupancyGridV1
    """
    def __init__(self) -> None: 
        """
        Constructor
        """
    @staticmethod
    def deserialize(arg0: numpy.ndarray[numpy.uint8]) -> ODSOccupancyGridV1: 
        """
        Deserialize ODSOccupancyGridV1 Buffer
        """
    @property
    def height(self) -> int:
        """
                 number of grid cells
              

        :type: int
        """
    @property
    def image(self) -> numpy.ndarray:
        """
                array of width * height
              

        :type: numpy.ndarray
        """
    @property
    def timestamp_ns(self) -> int:
        """
                timestamp of the grid 
              

        :type: int
        """
    @property
    def transform_cell_center_to_user(self) -> typing.List[float[6]]:
        """
                 values of matrix 2x3
                 affine mapping between grid cell and user coordinate system
                 e.g, multiplying the matrix with [0,0,1] gives the user coordinate
                 of the center of upper left cell
              

        :type: typing.List[float[6]]
        """
    @property
    def width(self) -> int:
        """
                number of grid cells
              

        :type: int
        """
    pass
class RGBInfoV1():
    """
    Class for managing an instance of an struct RGBInfoV1
    """
    def __init__(self) -> None: 
        """
        Constructor
        """
    @staticmethod
    def deserialize(arg0: numpy.ndarray[numpy.uint8]) -> RGBInfoV1: 
        """
        Deserialize RGB_INFO Buffer to RGBInfoV1 struct
        """
    @property
    def exposure_time(self) -> float:
        """
                Actual exposure time of the 2D image
              

        :type: float
        """
    @property
    def extrinsic_optic_to_user(self) -> ExtrinsicOpticToUser:
        """
                Extrinsic optic paramter of the 2D head
              

        :type: ExtrinsicOpticToUser
        """
    @property
    def frame_counter(self) -> int:
        """
                Frame count, The frame counter is initialized to 0 at the initialization
              

        :type: int
        """
    @property
    def intrinsic_calibration(self) -> Calibration:
        """
                Intrinsic Calibration parameters
              

        :type: Calibration
        """
    @property
    def inverse_intrinsic_calibration(self) -> Calibration:
        """
                Inverse intrinsic Calibration parameters
              

        :type: Calibration
        """
    @property
    def timestamp_ns(self) -> int:
        """
                The timestamp of the 2D image, given in nano second
              

        :type: int
        """
    @property
    def version(self) -> int:
        """
                Version of the RGB_INFO data
              

        :type: int
        """
    pass
class TOFInfoV3():
    """
    Class for managing an instance of an struct TofInfoV3
    """
    def __init__(self) -> None: 
        """
        Constructor
        """
    @staticmethod
    def deserialize(arg0: numpy.ndarray[numpy.uint8]) -> TOFInfoV3: 
        """
        Deserialize TOF_INFO Buffer to ToFInfoV3 struct.
        """
    @property
    def amp_normalization_factors(self) -> typing.List[float[3]]:
        """
                  Amplitude normalization factors for the individual exposure times
              

        :type: typing.List[float[3]]
        """
    @property
    def amplitude_resolution(self) -> float:
        """
                 Resolution of the amplitude buffer
              

        :type: float
        """
    @property
    def distance_resolution(self) -> float:
        """
                Resolution of distance buffer per digit[m]
              

        :type: float
        """
    @property
    def exposure_times_s(self) -> typing.List[int[3]]:
        """
                 Actual exposure times of a single phase image [seconds]
              

        :type: typing.List[int[3]]
        """
    @property
    def exposure_timestamps_ns(self) -> typing.List[int[3]]:
        """
                 The timestamp of the individual exposure time [nano seconds]
              

        :type: typing.List[int[3]]
        """
    @property
    def extrinsic_optic_to_user(self) -> ExtrinsicOpticToUser:
        """
                Extrinsic optic parameter to user
              

        :type: ExtrinsicOpticToUser
        """
    @property
    def illu_temperature(self) -> float:
        """
                 Illumination temperature
              

        :type: float
        """
    @property
    def imager(self) -> typing.List[str[32]]:
        """
                Imager type
              

        :type: typing.List[str[32]]
        """
    @property
    def intrinsic_calibration(self) -> Calibration:
        """
                Intrinsic calibration parameters
              

        :type: Calibration
        """
    @property
    def inverse_intrinsic_calibration(self) -> Calibration:
        """
                Inverse intrinsic calibration parameters
              

        :type: Calibration
        """
    @property
    def mode(self) -> typing.List[str[32]]:
        """
                Mode of the head
              

        :type: typing.List[str[32]]
        """
    @property
    def version(self) -> int:
        """
                 Version of the TOF_INFO data
              

        :type: int
        """
    pass
class TOFInfoV4(TOFInfoV3):
    """
    Class for managing an instance of an struct TofInfoV4
    """
    def __init__(self) -> None: 
        """
        Constructor
        """
    @staticmethod
    def deserialize(arg0: numpy.ndarray[numpy.uint8]) -> TOFInfoV4: 
        """
        Deserialize TOF_INFO buffer to ToFInfoV4
        """
    @property
    def measurement_block_index(self) -> int:
        """
                 Current measurement block index (range 0 to N-1, where N is the number of sub-modes).
                 This identifies the currently used sub-mode in cyclic modes.
                 In non-cyclic modes this value is always 0.
              

        :type: int
        """
    @property
    def measurement_range_max(self) -> float:
        """
                 Current maximum measurement range [m].
                 The value is based on the camera-individual ToF calibration.
                 It is influenced by temperature.
              

        :type: float
        """
    @property
    def measurement_range_min(self) -> float:
        """
                Current minimum measurement range [m].
                The value is based on the camera-individual ToF calibration.
                It is influenced by temperature.
              

        :type: float
        """
    pass
