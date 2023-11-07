import os
import sys
from typing import Any, List, Optional, Tuple, TypedDict

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
    aruco_ids: Optional[npt.NDArray[np.int32]]
    aruco_corners: Optional[Tuple[npt.NDArray[np.float32]]]
    charuco_ids: Optional[npt.NDArray[np.int32]]
    charuco_corners: Optional[Tuple[npt.NDArray[np.float32]]]


class Charuco:
    def __init__(
        self,
        dictionary: int,
        n_squares_x: int,
        n_squares_y: int,
        marker_length: float,
        square_length: float,
        legacy_pattern: bool = False,
    ) -> None:
        """
        Initialize a Charuco calibration object with the specified parameters.

        Parameters
        ----------
        dictionary : int
            The Aruco dictionary to use for marker detection.
        n_squares_x : int
            The number of squares along the X-axis on the Charuco board.
        n_squares_y : int
            The number of squares along the Y-axis on the Charuco board.
        marker_length : float
            The length of Aruco markers used on the board.
        square_length : float
            The length of squares on the Charuco board.

        Returns
        -------
        None
        """
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
        self.charuco_board.setLegacyPattern(legacy_pattern)
        self.charuco_detector = cv2.aruco.CharucoDetector(self.charuco_board)

    @staticmethod
    def sharp(
        image: npt.NDArray[np.uint8],
        kernel: Optional[npt.NDArray[np.float32]] = None,
    ) -> npt.NDArray[np.uint8]:
        """
        Apply a sharpening filter to the input image.

        Parameters
        ----------
        image : npt.NDArray[np.uint8]
            The input image to be sharpened.

        Returns
        -------
        npt.NDArray[np.uint8]
            The sharpened image.

        Notes
        -----
        The sharpening filter used is a kernel with the following weights:

        [[ 0, -1,  0],
         [-1,  5, -1],
         [ 0, -1,  0]]
        """
        if kernel is None:
            kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        return cv2.filter2D(src=image, ddepth=-1, kernel=kernel)

    def detect(self, image: npt.NDArray[np.uint8]) -> Tuple[int, CharucoDetection]:
        """
        Detects Aruco and Charuco markers in the given image.

        Parameters
        ----------
        image : npt.NDArray[np.uint8]
            The input image for marker detection.

        Returns
        -------
        Tuple[int, CharucoDetection]
            A tuple containing the number of detected corners in Charuco markers and markers
            details.

        Notes
        -----
        This method sharpens the input image and then detects both Aruco and Charuco markers.
        Detected Aruco markers are refined for better accuracy using cornerSubPix.
        The function returns the number of detected corners in Charuco markers and their details.
        """
        sharped = self.sharp(image=image)
        aruco_corners, aruco_ids, _ = self.aruco_detector.detectMarkers(image=sharped)
        criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 100, 0.001)
        for aruco_corner in aruco_corners:
            aruco_corner = cv2.cornerSubPix(
                sharped, aruco_corner, (3, 3), (-1, -1), criteria
            )
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
        self,
        detections: List[Tuple[npt.NDArray[np.uint8], CharucoDetection]],
    ) -> CameraCalibration:
        """
        Calibrates the camera using Charuco marker detections.

        It collects data from multiple image and detection pairs and computes the camera
        intrinsic parameters and distortion coefficients. The calibration results are returned
        as a CameraCalibration object.

        Parameters
        ----------
        detections : List[Tuple[npt.NDArray[np.uint8], CharucoDetection]]
            A list of image and marker detection tuples.

        Returns
        -------
        CameraCalibration
            The camera calibration results.
        """
        all_corners_flatten: List[npt.NDArray[Any]] = []
        all_ids_flatten: List[npt.NDArray[Any]] = []

        for _, detection in detections:
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
    current: CharucoDetection,
    last: CharucoDetection,
    distance_threshold: int = 100,
) -> bool:
    """
    Checks if the Charuco board has moved based on the detected Aruco markers.

    This function compares the detected Aruco markers between the current and previous Charuco
    marker detections. If the movement exceeds the specified distance threshold, it is considered
    that the Charuco board has moved.

    Parameters
    ----------
    current : CharucoDetection
        The Charuco marker detection in the current frame.
    last : CharucoDetection
        The Charuco marker detection in the previous frame.
    distance_threshold : int, optional
        The threshold distance for movement detection. Defaults to 100.

    Returns
    -------
    bool
        True if the board has moved, False otherwise.
    """
    if (
        current["aruco_ids"] is not None
        and last["aruco_ids"] is not None
        and current["aruco_corners"] is not None
        and last["aruco_corners"] is not None
    ):
        curr_ids = current["aruco_ids"].flatten()
        last_ids = last["aruco_ids"].flatten()
        for i, curr_id in enumerate(curr_ids):
            if curr_id in last_ids:
                (j,) = np.where(last_ids == curr_id)
                distance = cv2.norm(
                    last["aruco_corners"][j[0]] - current["aruco_corners"][i]
                )
                if distance > distance_threshold:
                    return True
    return False


def draw_detection(
    image: npt.NDArray[np.uint8],
    detection: CharucoDetection,
) -> npt.NDArray[np.uint8]:
    """
    Draws detected Aruco and Charuco markers on the input image.

    This function takes an image and a Charuco marker detection and draws the detected Aruco
    markers and Charuco corners on the image. The modified image is returned with the markers and
    corners visualized.

    Parameters
    ----------
    image : npt.NDArray[np.uint8]
        The input image to which markers and corners will be drawn.
    detection : CharucoDetection
        A dictionary containing the detected Aruco and Charuco markers.

    Returns
    -------
    npt.NDArray[np.uint8]
        The modified image with markers and corners visualized.
    """
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
    """
    Saves images from detected Charuco markers to the specified directory.

    This function saves images containing detected Charuco markers to the specified directory with
    filenames in the format "000.png", "001.png", and so on.

    Parameters
    ----------
    detections : List[Tuple[npt.NDArray[np.uint8], CharucoDetection]]
        A list of image and Charuco marker detection tuples.
    data_dir : str
        The directory where the images will be saved.

    Returns
    -------
    None
    """
    for index, (image, _) in enumerate(detections):
        cv2.imwrite("{}/{:003}.png".format(data_dir, index), image)


def main() -> None:
    """
    The main function for performing camera calibration using Charuco markers.

    This function reads calibration options, captures images, detects markers, and calibrates
    the camera using Charuco markers.

    Parameters
    ----------
    None (program arguments are passed via the command line or from the default options file).

    Returns
    -------
    None

    Notes
    -----
    - Ensure that the required dependencies are installed.
    - If the URI for camera input is provided, the program subscribes to a specific topic and
      captures images until the desired number of samples is achieved.
    - If no URI is provided, the program processes images from a specified folder.
    - Detected Charuco markers and camera calibration results are saved to the specified
      directory.
    - The program can also save images containing detected Charuco markers if specified in the
      options.
    """
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
        legacy_pattern=options.intrinsic.legacy_pattern,
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
            if counter > (
                (options.intrinsic.n_squares_x * options.intrinsic.n_squares_y) / 2
            ):
                if len(detections) > 0:
                    if board_moved(current=detection, last=detections[-1][1]):
                        logger.info("Board moved, saved detection")
                        detections.append((array, detection))
                else:
                    logger.info("Saved first detection")
                    detections.append((array, detection))
            else:
                if counter != 0:
                    logger.warn("Requires at least half corners, corners={}", counter)
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
        while images.isOpened():
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
    if len(detections) >= options.intrinsic.samples:
        logger.info("Calibrating camera id={}", options.camera_id)
        calibration = charuco.calibrate_camera(detections=detections)
        calibration.id = options.camera_id
        os.makedirs(options.data_dir)
        with open(
            file=os.path.join(options.data_dir, f"{options.camera_id}.json"),
            mode="w",
            encoding="utf-8",
        ) as file:
            content = MessageToJson(calibration, indent=2)
            file.write(content)

        if options.save_images:
            save_images(data_dir=options.data_dir, detections=detections)
    else:
        logger.warn(
            "Did not detect enough charuco boards: {}/{}",
            len(detections),
            options.intrinsic.samples,
        )


if __name__ == "__main__":
    main()
