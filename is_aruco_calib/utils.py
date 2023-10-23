import cv2
import numpy as np
import numpy.typing as npt
from is_msgs.common_pb2 import DataType, Tensor
from is_msgs.image_pb2 import Image
from google.protobuf.message import Message
from google.protobuf.json_format import Parse, ParseError
from is_wire.core import Logger

def array2tensor(array: np.ndarray) -> Tensor:
    tensor = Tensor()
    rows = tensor.shape.dims.add()
    rows.size = array.shape[0]
    rows.name = "rows"
    cols = tensor.shape.dims.add()
    cols.size = array.shape[1]
    cols.name = "cols"
    if array.dtype == np.int32:
        tensor.type = DataType.Value("INT32_TYPE")
        tensor.ints32.extend(array.ravel().tolist())
    elif array.dtype == np.int64:
        tensor.type = DataType.Value("INT64_TYPE")
        tensor.ints64.extend(array.ravel().tolist())
    elif array.dtype == np.float32:
        tensor.type = DataType.Value("FLOAT_TYPE")
        tensor.floats.extend(array.ravel().tolist())
    elif array.dtype == np.float64:
        tensor.type = DataType.Value("DOUBLE_TYPE")
        tensor.doubles.extend(array.ravel().tolist())
    return tensor


def image2array(image: Image) -> npt.NDArray[np.uint8]:
    buffer = np.frombuffer(image.data, dtype=np.uint8)
    array = cv2.imdecode(buffer, flags=cv2.IMREAD_GRAYSCALE)
    return array

def load_json(filename: str, logger: Logger, schema: Message) -> Message:
    try:
        with open(file=filename, mode="r", encoding="utf-8") as file:
            try:
                options = Parse(file.read(), schema())
                logger.info("Options: \n{}", options)
                return options
            except ParseError as ex:
                logger.critical("Unable to load options from '{}'. \n{}", filename, ex)
    except FileNotFoundError as ex:
        logger.critical("Unable to open file '{}'.  \n{}", filename, ex)
