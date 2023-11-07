from typing import Any

import cv2
import numpy as np
import numpy.typing as npt
from google.protobuf.json_format import Parse, ParseError
from google.protobuf.message import Message
from is_msgs.common_pb2 import DataType, Tensor
from is_msgs.image_pb2 import Image
from is_wire.core import Logger


def tensor2array(tensor: Tensor) -> npt.NDArray[Any]:
    """
    Convert a Tensor protobuf message to a NumPy array.

    Parameters
    ----------
    array : Tensor
        The Tensor message to be converted to a Numpy array.

    Returns
    -------
    npt.NDArray[Any]
        The resulting Numpy array.
    """
    if len(tensor.shape.dims) != 2 or tensor.shape.dims[0].name != "rows":
        return np.array([])
    shape = (tensor.shape.dims[0].size, tensor.shape.dims[1].size)
    if tensor.type == DataType.Value("INT32_TYPE"):
        return np.array(tensor.ints32, dtype=np.int32, copy=False).reshape(shape)
    if tensor.type == DataType.Value("INT64_TYPE"):
        return np.array(tensor.ints64, dtype=np.int64, copy=False).reshape(shape)
    if tensor.type == DataType.Value("FLOAT_TYPE"):
        return np.array(tensor.floats, dtype=np.float32, copy=False).reshape(shape)
    if tensor.type == DataType.Value("DOUBLE_TYPE"):
        return np.array(tensor.doubles, dtype=np.float64, copy=False).reshape(shape)
    return np.array([])


def array2tensor(array: npt.NDArray[Any]) -> Tensor:
    """
    Convert a NumPy array to a Tensor protobuf message.

    Parameters
    ----------
    array : npt.NDArray[Any]
        The NumPy array to be converted to a Tensor.

    Returns
    -------
    Tensor
        The resulting Tensor message.
    """
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
    """
    Convert an Image protobuf message to a NumPy array.

    Parameters
    ----------
    array : Image
        The Image protobuf message to be converted to a NumPy array.

    Returns
    -------
    npt.NDArray[np.uint8]
        The NumPy array representing the image in grayscale.
    """
    buffer = np.frombuffer(image.data, dtype=np.uint8)
    array = cv2.imdecode(buffer, flags=cv2.IMREAD_GRAYSCALE)
    return array


def load_json(filename: str, logger: Logger, schema: Message) -> Message:
    """
    Load JSON data from a file and parse it into a protobuf message.

    This function loads JSON data from the specified file, parses it into a protobuf message
    based on the provided schema, and returns the resulting message.

    Parameters
    ----------
    filename : str
        The path to the JSON file to be loaded and parsed.
    logger : Logger
        An instance of the Logger for logging messages.
    schema : Message
        The protobuf message schema to parse the JSON data into.

    Returns
    -------
    Message
        The parsed protobuf message.
    """
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
