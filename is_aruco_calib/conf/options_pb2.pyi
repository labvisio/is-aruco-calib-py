from is_msgs import validate_pb2 as _validate_pb2
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar, Mapping, Optional, Union

DESCRIPTOR: _descriptor.FileDescriptor
DICT_4X4_100: ArucoDictionary
DICT_4X4_1000: ArucoDictionary
DICT_4X4_250: ArucoDictionary
DICT_4X4_50: ArucoDictionary
DICT_5X5_100: ArucoDictionary
DICT_5X5_1000: ArucoDictionary
DICT_5X5_250: ArucoDictionary
DICT_5X5_50: ArucoDictionary
DICT_6X6_100: ArucoDictionary
DICT_6X6_1000: ArucoDictionary
DICT_6X6_250: ArucoDictionary
DICT_6X6_50: ArucoDictionary

class CalibrationOptions(_message.Message):
    __slots__ = ["camera_id", "data_dir", "extrinsic", "intrinsic", "save_images", "topic", "uri", "world_id"]
    CAMERA_ID_FIELD_NUMBER: ClassVar[int]
    DATA_DIR_FIELD_NUMBER: ClassVar[int]
    EXTRINSIC_FIELD_NUMBER: ClassVar[int]
    INTRINSIC_FIELD_NUMBER: ClassVar[int]
    SAVE_IMAGES_FIELD_NUMBER: ClassVar[int]
    TOPIC_FIELD_NUMBER: ClassVar[int]
    URI_FIELD_NUMBER: ClassVar[int]
    WORLD_ID_FIELD_NUMBER: ClassVar[int]
    camera_id: int
    data_dir: str
    extrinsic: ExtrinsicCalibrationOptions
    intrinsic: IntrinsicCalibrationOptions
    save_images: bool
    topic: str
    uri: str
    world_id: int
    def __init__(self, uri: Optional[str] = ..., topic: Optional[str] = ..., data_dir: Optional[str] = ..., camera_id: Optional[int] = ..., world_id: Optional[int] = ..., intrinsic: Optional[Union[IntrinsicCalibrationOptions, Mapping]] = ..., extrinsic: Optional[Union[ExtrinsicCalibrationOptions, Mapping]] = ..., save_images: bool = ...) -> None: ...

class CreateArucoOptions(_message.Message):
    __slots__ = ["border_bits", "dictionary", "marker_id", "marker_length"]
    BORDER_BITS_FIELD_NUMBER: ClassVar[int]
    DICTIONARY_FIELD_NUMBER: ClassVar[int]
    MARKER_ID_FIELD_NUMBER: ClassVar[int]
    MARKER_LENGTH_FIELD_NUMBER: ClassVar[int]
    border_bits: int
    dictionary: ArucoDictionary
    marker_id: int
    marker_length: int
    def __init__(self, dictionary: Optional[Union[ArucoDictionary, str]] = ..., marker_id: Optional[int] = ..., marker_length: Optional[int] = ..., border_bits: Optional[int] = ...) -> None: ...

class CreateCharucoOptions(_message.Message):
    __slots__ = ["border_bits", "dictionary", "marker_length", "n_squares_x", "n_squares_y", "square_length"]
    BORDER_BITS_FIELD_NUMBER: ClassVar[int]
    DICTIONARY_FIELD_NUMBER: ClassVar[int]
    MARKER_LENGTH_FIELD_NUMBER: ClassVar[int]
    N_SQUARES_X_FIELD_NUMBER: ClassVar[int]
    N_SQUARES_Y_FIELD_NUMBER: ClassVar[int]
    SQUARE_LENGTH_FIELD_NUMBER: ClassVar[int]
    border_bits: int
    dictionary: ArucoDictionary
    marker_length: float
    n_squares_x: int
    n_squares_y: int
    square_length: float
    def __init__(self, dictionary: Optional[Union[ArucoDictionary, str]] = ..., marker_length: Optional[float] = ..., square_length: Optional[float] = ..., n_squares_x: Optional[int] = ..., n_squares_y: Optional[int] = ..., border_bits: Optional[int] = ...) -> None: ...

class CreateMarker(_message.Message):
    __slots__ = ["aruco", "charuco"]
    ARUCO_FIELD_NUMBER: ClassVar[int]
    CHARUCO_FIELD_NUMBER: ClassVar[int]
    aruco: CreateArucoOptions
    charuco: CreateCharucoOptions
    def __init__(self, aruco: Optional[Union[CreateArucoOptions, Mapping]] = ..., charuco: Optional[Union[CreateCharucoOptions, Mapping]] = ...) -> None: ...

class ExtrinsicCalibrationOptions(_message.Message):
    __slots__ = ["dictionary", "marker_id", "marker_length", "offset"]
    DICTIONARY_FIELD_NUMBER: ClassVar[int]
    MARKER_ID_FIELD_NUMBER: ClassVar[int]
    MARKER_LENGTH_FIELD_NUMBER: ClassVar[int]
    OFFSET_FIELD_NUMBER: ClassVar[int]
    dictionary: ArucoDictionary
    marker_id: int
    marker_length: float
    offset: float
    def __init__(self, dictionary: Optional[Union[ArucoDictionary, str]] = ..., marker_id: Optional[int] = ..., marker_length: Optional[float] = ..., offset: Optional[float] = ...) -> None: ...

class IntrinsicCalibrationOptions(_message.Message):
    __slots__ = ["dictionary", "marker_length", "n_squares_x", "n_squares_y", "samples", "square_length"]
    DICTIONARY_FIELD_NUMBER: ClassVar[int]
    MARKER_LENGTH_FIELD_NUMBER: ClassVar[int]
    N_SQUARES_X_FIELD_NUMBER: ClassVar[int]
    N_SQUARES_Y_FIELD_NUMBER: ClassVar[int]
    SAMPLES_FIELD_NUMBER: ClassVar[int]
    SQUARE_LENGTH_FIELD_NUMBER: ClassVar[int]
    dictionary: ArucoDictionary
    marker_length: float
    n_squares_x: int
    n_squares_y: int
    samples: int
    square_length: float
    def __init__(self, dictionary: Optional[Union[ArucoDictionary, str]] = ..., marker_length: Optional[float] = ..., square_length: Optional[float] = ..., n_squares_x: Optional[int] = ..., n_squares_y: Optional[int] = ..., samples: Optional[int] = ...) -> None: ...

class ArucoDictionary(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
