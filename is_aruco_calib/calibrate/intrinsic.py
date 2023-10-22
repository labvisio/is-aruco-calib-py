import sys
import json
from typing import List, TypedDict, Any, Tuple

import cv2
import numpy as np
import numpy.typing as npt

from google.protobuf.json_format import Parse, ParseError
from google.protobuf.timestamp_pb2 import Timestamp
from google.protobuf.json_format import MessageToJson

from is_msgs.camera_pb2 import CameraCalibration
from is_msgs.common_pb2 import Tensor, DataType
from is_msgs.image_pb2 import Image

from is_wire.core import Logger, Channel, Subscription

from is_aruco_calib.conf.options_pb2 import CalibrationOptions

class CharucoDetection(TypedDict):
    aruco_ids: npt.NDArray[Any]
    aruco_corners: npt.NDArray[Any]
    charuco_ids: npt.NDArray[Any]
    charuco_corners: npt.NDArray[Any]


class Charuco:
    def __init__(self,
                 dictionary: int,
                 n_squares_x: int,
                 n_squares_y: int,
                 marker_length: float,
                 square_length: float) -> None:
        # aruco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict=dictionary)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(
            dictionary=self.aruco_dict,
            detectorParams=self.aruco_params,
        )

        # charuco
        self.charuco_board = cv2.aruco.CharucoBoard(
            size=(n_squares_x, n_squares_y),
            squareLength=square_length,
            markerLength=marker_length,
            dictionary=self.aruco_dict,
        )

    @staticmethod
    def sharp(image: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint8]:
        kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
        return cv2.filter2D(src=image, depth=-1, kernel=kernel)
    
    @staticmethod
    def array2tensor(array: np.ndarray) -> Tensor:
        tensor = Tensor()
        rows = tensor.shape.dims.add()
        rows.size = array.shape[0]
        rows.name = "rows"
        cols = tensor.shape.dims.add()
        cols.size = array.shape[1]
        cols.name = "cols"
        if array.dtype == np.int32:
            tensor.type = DataType.Value('INT32_TYPE')
            tensor.ints32.extend(array.ravel().tolist())
        elif array.dtype == np.int64:
            tensor.type = DataType.Value('INT64_TYPE')
            tensor.ints64.extend(array.ravel().tolist())
        elif array.dtype == np.float32:
            tensor.type = DataType.Value('FLOAT_TYPE')
            tensor.floats.extend(array.ravel().tolist())
        elif array.dtype == np.float64:
            tensor.type = DataType.Value('DOUBLE_TYPE')
            tensor.doubles.extend(array.ravel().tolist())
        return tensor

    def detect(self, image: npt.NDArray[np.uint8]) -> Tuple[int, CharucoDetection]:
        sharped = self.sharp(image=image)
        aruco_corners, aruco_ids, _ = self.aruco_detector.detectMarkers(image=sharped)
        criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 100, .001)
        for aruco_corner in aruco_corners:
            cv2.cornerSubPix(sharped, aruco_corner, (3, 3), (-1, -1), criteria)
        counter, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            markerCorners=aruco_corners,
            markerIds=aruco_ids,
            image=sharped,
            board=self.charuco_board,
        )
        return counter, CharucoDetection(
            aruco_corners=aruco_corners,
            aruco_ids=aruco_ids,
            charuco_corners=charuco_corners,
            charuco_ids=charuco_ids,
        )

    def calibrate_camera(self, detections: List[Tuple[npt.NDArray[np.uint8], CharucoDetection]]) -> CameraCalibration:
        
        # all_corners_flatten: List[npt.NDArray[Any]] = []
        # all_ids_flatten: List[npt.NDArray[Any]] = []
        # markers_per_frame: List[npt.NDArray[Any]] = []

        # for detection in detections:
        #     if detection["aruco_ids"] is None:
        #         continue
        #     all_corners_flatten.append(detection["aruco_corners"])
        #     all_ids_flatten.append(detection["aruco_ids"])
        #     markers_per_frame.append(len(detection["aruco_ids"]))

        # error, camera_matrix, distortion, _, _ = cv2.aruco.calibrateCameraAruco(
        #     corners=all_corners_flatten,
        #     ids=all_ids_flatten,
        #     counter=markers_per_frame,
        #     board=self.charuco_board,
        # )

        all_corners_flatten: List[npt.NDArray[Any]] = []
        all_ids_flatten: List[npt.NDArray[Any]] = []

        for _, detection in detections:
            if None in detection.values():
                continue
            all_corners_flatten.append(detection["charuco_corners"])
            all_ids_flatten.append(detection["charuco_ids"])

        error, camera_matrix, distortion_coeffs, _, _ = cv2.aruco.calibrateCameraCharuco(
            charucoCorners=all_corners_flatten,
            charucoIds=all_ids_flatten,
            board=self.board,
            imageSize=detections[0][0].shape,
            cameraMatrix=None,
        )
        calibration = CameraCalibration()
        calibration.intrinsic.CopyFrom(self.array2tensor(array=camera_matrix))
        calibration.distortion.CopyFrom(self.array2tensor(array=distortion_coeffs))
        calibration.error = error
        calibration.resolution.heigth = detections[0][0].shape[0]
        calibration.resolution.width = detections[0][0].shape[1]
        timestamp = Timestamp()
        timestamp.GetCurrentTime()
        calibration.calibrated_at(timestamp)
        return calibration

def load_json(filename: str, logger: Logger) -> CalibrationOptions:
    try:
        with open(filename, "r", encoding="utf-8") as file:
            try:
                options = Parse(file.read(), CalibrationOptions())
                logger.info("Options: \n{}", options)
                return options
            except ParseError as ex:
                logger.critical("Unable to load options from '{}'. \n{}", filename, ex)
    except FileNotFoundError as ex:
        logger.critical("Unable to open file '{}'.  \n{}", filename, ex)


def image2array(image: Image) -> npt.NDArray[np.uint8]:
        buffer = np.frombuffer(image.data, dtype=np.uint8)
        array = cv2.imdecode(buffer, flags=cv2.IMREAD_GRAYSCALE)
        return array


def board_moved(current: CharucoDetection, last: CharucoDetection, distance_threshold: int = 100) -> bool:
    curr_ids = current["aruco_ids"].flatten() 
    last_ids = last["aruco_ids"].flatten()
    for curr_id in curr_ids:
        if curr_id in last_ids:
            distance = cv2.norm(last["aruco_corners"][0] - current["aruco_corners"][0])
            if distance > distance_threshold:
                return True
    return False

def draw_detection(image: npt.NDArray[np.unit8], detection: CharucoDetection):
    image_copy = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if detection["aruco_corners"] is not None:
        image_copy = cv2.aruco.drawDetectedMarkers(
            image=image_copy,
            corners=detection["aruco_corners"],
            ids=detection["aruco_ids"],
        )
    if detection["charuco_corners"] is not None:
        image_copy = cv2.aruco.drawDetectedCornersCharuco(
            image=image_copy,
            charucoCorners=detection["charuco_corners"],
            charucoIds=detection["charuco_ids"],
        )
    return image_copy

def main():
    logger = Logger("IntrinsicCalibration")
    filename = sys.argv[1] if len(sys.argv) > 1 else "options.json"
    options = load_json(filename=filename, logger=logger)
    charuco = Charuco(
        dictionary=options.intrinsic.dictionary,
        n_squares_x=options.intrinsic.n_squares_x,
        n_squares_y=options.intrinsic.n_squares_y,
        square_length=options.intrinsic.square_length,
        marker_length=options.intrinsic.marker_length,
    )
    detections: List[Tuple[npt.NDArray[np.uint8], CharucoDetection]] = []
    if options.uri:
        channel = Channel(uri=options.uri, exchange="is")
        subscrition = Subscription(channel=channel, name="IntrinsicCalibration")
        subscrition.subscribe(options.topic.format(options.camera_id))
        while len(detections) != options.intrinsic.samples:
            message = channel.consume()
            image = message.unpack(Image)
            array = image2array(image=image)
            counter, detection = charuco.detect(image=array)
            if counter < ((options.intrinsic.n_squares_x * options.intrinsic.n_squares_y) / 2):
                logger.warn("Requires at least half")
                continue
            if len(detections) > 0:
                if board_moved(current=detection, last=detections[-1]):
                    logger.info("Board moved, saved detection")
                    detections.append(detection)
            else:
                logger.info("Saved first detection")
                detections.append(detection)
            image_copy = draw_detection(image=array, detection=detection)
            cv2.imshow("Detection", image_copy)
    else:
        logger.info("Using folder={}, and searching for *.png files", options.data_dir)
        images = cv2.VideoCapture(f"{options.data_dir}/%d.png")
        while images.isOpened:
            ret, frame = images.read()
            if not ret:
                logger.warn("Can't read image, skipping...")
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detection = charuco.detect(image=gray)
            detections.append(detection)

    logger.info("Calibrating camera id={}", options.camera_id)
    calibration = charuco.calibrate_camera(detections=detections)
    calibration.id = options.camera_id

    with open(f"{options.data_dir}/{options.camera_id}.json", 'w') as file:
        content = MessageToJson(calibration, indent=4)
        file.write(content)


if __name__ == "__main__":
    main()
