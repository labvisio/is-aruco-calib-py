import sys
import os
from typing import Any, List, Tuple, TypedDict

import cv2
import numpy as np
import numpy.typing as npt
from google.protobuf.json_format import MessageToJson
from google.protobuf.timestamp_pb2 import Timestamp
from is_msgs.camera_pb2 import CameraCalibration
from is_msgs.image_pb2 import Image
from is_wire.core import Channel, Logger, Subscription

from is_aruco_calib.conf.options_pb2 import CalibrationOptions
from is_aruco_calib.utils import array2tensor, image2array, load_json


class CharucoDetection(TypedDict):
    aruco_ids: npt.NDArray[Any]
    aruco_corners: npt.NDArray[Any]
    charuco_ids: npt.NDArray[Any]
    charuco_corners: npt.NDArray[Any]


class Charuco:
    def __init__(
        self,
        dictionary: int,
        n_squares_x: int,
        n_squares_y: int,
        marker_length: float,
        square_length: float,
    ) -> None:
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict=dictionary)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(
            dictionary=self.aruco_dict,
            detectorParams=self.aruco_params,
        )
        self.charuco_board = cv2.aruco.CharucoBoard(
            size=(n_squares_x, n_squares_y),
            squareLength=square_length,
            markerLength=marker_length,
            dictionary=self.aruco_dict,
        )
        # self.charuco_board.setLegacyPattern(True)
        self.charuco_detector = cv2.aruco.CharucoDetector(self.charuco_board)

    @staticmethod
    def sharp(image: npt.NDArray[np.uint8]) -> npt.NDArray[np.uint8]:
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        return cv2.filter2D(src=image, ddepth=-1, kernel=kernel)

    def detect(self, image: npt.NDArray[np.uint8]) -> Tuple[int, CharucoDetection]:
        sharped = self.sharp(image=image)
        aruco_corners, aruco_ids, _ = self.aruco_detector.detectMarkers(image=sharped)
        criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 100, 0.001)
        for aruco_corner in aruco_corners:
            aruco_corner = cv2.cornerSubPix(
                sharped, aruco_corner, (3, 3), (-1, -1), criteria
            )
        if aruco_corners is not None or aruco_ids is not None:
            detection = CharucoDetection(
                aruco_corners=aruco_corners,
                aruco_ids=aruco_ids,
                charuco_corners=None,
                charuco_ids=None,
            )
        charuco_corners, charuco_ids, _, _ = self.charuco_detector.detectBoard(
            image=sharped,
            markerCorners=aruco_corners,
            markerIds=aruco_ids,
        )
        if charuco_ids is None or charuco_corners is None:
            return 0, detection
        detection["charuco_corners"] = charuco_corners
        detection["charuco_ids"] = charuco_ids
        return len(charuco_corners), detection

    def calibrate_camera(
        self, detections: List[Tuple[npt.NDArray[np.uint8], CharucoDetection]]
    ) -> CameraCalibration:
        all_corners_flatten: List[npt.NDArray[Any]] = []
        all_ids_flatten: List[npt.NDArray[Any]] = []

        for (_, detection) in detections:
            all_corners_flatten.append(detection["charuco_corners"])
            all_ids_flatten.append(detection["charuco_ids"])
        (
            error,
            camera_matrix,
            distortion_coeffs,
            _,
            _,
        ) = cv2.aruco.calibrateCameraCharuco(
            charucoCorners=all_corners_flatten,
            charucoIds=all_ids_flatten,
            board=self.charuco_board,
            imageSize=detections[0][0].shape,
            cameraMatrix=None,
            distCoeffs=None,
        )
        calibration = CameraCalibration()
        calibration.intrinsic.CopyFrom(array2tensor(array=camera_matrix))
        calibration.distortion.CopyFrom(array2tensor(array=distortion_coeffs))
        calibration.error = error
        calibration.resolution.height = detections[0][0].shape[0]
        calibration.resolution.width = detections[0][0].shape[1]
        timestamp = Timestamp()
        timestamp.GetCurrentTime()
        calibration.calibrated_at.CopyFrom(timestamp)
        return calibration


def board_moved(
    current: CharucoDetection, last: CharucoDetection, distance_threshold: int = 100
) -> bool:
    curr_ids = current["aruco_ids"].flatten()
    last_ids = last["aruco_ids"].flatten()
    for i, curr_id in enumerate(curr_ids):
        if curr_id in last_ids:
            j, = np.where(last_ids == curr_id)
            distance = cv2.norm(last["aruco_corners"][j[0]] - current["aruco_corners"][i])
            if distance > distance_threshold:
                return True
    return False


def draw_detection(image: npt.NDArray[np.uint8], detection: CharucoDetection):
    image_copy = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if detection["aruco_corners"] is not None:
        image_copy = cv2.aruco.drawDetectedMarkers(
            image=image_copy.copy(),
            corners=detection["aruco_corners"],
            ids=detection["aruco_ids"],
        )
    if detection["charuco_corners"] is not None:
        image_copy = cv2.aruco.drawDetectedCornersCharuco(
            image=image_copy.copy(),
            charucoCorners=detection["charuco_corners"],
            charucoIds=detection["charuco_ids"],
            cornerColor=(255, 125, 125),
        )
    return image_copy


def save_images(
    detections: List[Tuple[npt.NDArray[np.uint8], CharucoDetection]],
    data_dir: str,
) -> None:
    for index, (image, _) in enumerate(detections):
        cv2.imwrite("{}/{:003}.png".format(data_dir, index), image)


def main():
    logger = Logger("IntrinsicCalibration")
    filename = sys.argv[1] if len(sys.argv) > 1 else "options.json"
    options = load_json(
        filename=filename,
        schema=CalibrationOptions,
        logger=logger,
    )
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
            if counter < (
                (options.intrinsic.n_squares_x * options.intrinsic.n_squares_y) / 2
            ):
                logger.warn("Requires at least half")
            else:
                if len(detections) > 0:
                    if board_moved(current=detection, last=detections[-1][1]):
                        logger.info("Board moved, saved detection")
                        detections.append((array, detection))
                else:
                    logger.info("Saved first detection")
                    detections.append((array, detection))
            image_copy = draw_detection(image=array, detection=detection)
            image_copy = cv2.putText(
                img=image_copy,
                text=f"{len(detections)}/{options.intrinsic.samples}",
                org=(10, 50),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=1.25,
                color=(0, 255, 0),
                thickness=2,
            )
            cv2.imshow("Detection", image_copy)
            if cv2.waitKey(1) == ord("q"):
                break
        cv2.destroyAllWindows()
    else:
        logger.info("Using folder={}, and searching for *.png files", options.data_dir)
        images = cv2.VideoCapture(f"{options.data_dir}/%3d.png")
        while images.isOpened:
            ret, frame = images.read()
            if not ret:
                logger.warn("Can't read image, skipping...")
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, detection = charuco.detect(image=gray)
            if len(detections) > 0:
                if board_moved(current=detection, last=detections[-1][1]):
                    logger.info("Board moved, saved detection")
                    detections.append((gray, detection))
            else:
                logger.info("Saved first detection")
                detections.append((gray, detection))

    logger.info("Calibrating camera id={}", options.camera_id)
    calibration = charuco.calibrate_camera(detections=detections)
    calibration.id = options.camera_id

    with open(
        file=os.path.join(options.data_dir, f"{options.camera_id}.json"),
        mode="w",
        encoding="utf-8",
    ) as file:
        content = MessageToJson(calibration, indent=2)
        file.write(content)

    if options.save_images:
        save_images(data_dir=options.data_dir, detections=detections)


if __name__ == "__main__":
    main()
