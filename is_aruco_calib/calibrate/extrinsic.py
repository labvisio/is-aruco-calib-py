import os
import sys

import cv2
import numpy as np
import numpy.typing as npt
from google.protobuf.json_format import MessageToJson
from is_msgs.camera_pb2 import CameraCalibration
from is_msgs.image_pb2 import Image
from is_wire.core import Channel, Logger, Subscription

from is_aruco_calib.conf.options_pb2 import CalibrationOptions
from is_aruco_calib.utils import (
    array2tensor,
    image2array,
    load_json,
    tensor2array,
)


def transformation_matrix(
    rvec: npt.NDArray[np.float32],
    tvec: npt.NDArray[np.float32],
) -> npt.NDArray[np.float32]:
    """
    Compute a transformation matrix from a rotation vector (rvec) and a translation vector
    (tvec).

    Parameters
    ----------
    rvec : npt.NDArray[np.float32]
        Rotation vector (3x1) representing rotation.

    tvec : npt.NDArray[np.float32]
        Translation vector (3x1) representing translation.

    Returns
    -------
    npt.NDArray[np.float32]
        A 4x4 transformation matrix representing both rotation and translation.
    """
    rmat, _ = cv2.Rodrigues(rvec)
    tvec = tvec.reshape(3, 1)
    mat = np.hstack((rmat, tvec))
    mat = np.vstack((mat, np.array([[0, 0, 0, 1]])))
    return mat


def main() -> None:
    """
    The main function for performing extrinsic camera calibration using Aruco markers.

    Reads intrinsic calibration options, captures images, detects markers, and calculates
    extrinsic transformation.

    Parameters
    ----------
    None (program arguments are passed via the command line or from the default options file).

    Returns
    -------
    None
    """
    logger = Logger("ExtrinsicCalibration")
    filename = sys.argv[1] if len(sys.argv) > 1 else "options.json"
    options = load_json(
        filename=filename,
        schema=CalibrationOptions,
        logger=logger,
    )
    channel = Channel(uri=options.uri, exchange="is")
    subscrition = Subscription(channel=channel, name="ExtrinsicCalibration")
    subscrition.subscribe(options.topic.format(options.camera_id))
    dictionary = cv2.aruco.getPredefinedDictionary(dict=options.extrinsic.dictionary)
    detector = cv2.aruco.ArucoDetector(dictionary=dictionary)
    calibration = load_json(
        filename=os.path.join(
            options.data_dir,
            f"{options.camera_id}.json",
        ),
        schema=CameraCalibration,
        logger=logger,
    )
    intrinsic = tensor2array(calibration.intrinsic)
    distortion = tensor2array(calibration.distortion)
    while True:
        message = channel.consume()
        image = message.unpack(Image)
        array = image2array(image=image)
        corners, ids, _ = detector.detectMarkers(image=array)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners=corners,
            markerLength=options.extrinsic.marker_length,
            cameraMatrix=intrinsic,
            distCoeffs=distortion,
        )
        image_copy = cv2.cvtColor(src=array, code=cv2.COLOR_GRAY2BGR)
        image_copy = cv2.aruco.drawDetectedMarkers(
            image=image_copy.copy(),
            corners=corners,
            ids=ids,
        )
        translation = None
        rotation = None
        detected_marker = False
        if ids is not None:
            ids_flatten = ids.flatten()
            for i in range(len(ids_flatten)):
                rvec = rvecs[i]
                tvec = tvecs[i]
                image_copy = cv2.drawFrameAxes(
                    image=image_copy,
                    cameraMatrix=intrinsic,
                    distCoeffs=distortion,
                    rvec=rvec,
                    tvec=tvec,
                    length=options.extrinsic.marker_length,
                )
            (j,) = np.where(ids_flatten == options.extrinsic.marker_id)
            if len(j) > 0:
                detected_marker = True
                translation = tvecs[j[0]]
                rotation = rvecs[j[0]]
                logger.info(
                    "marker_id={}, rotation={}, translation={}",
                    options.extrinsic.marker_id,
                    rotation,
                    translation,
                )
        cv2.imshow("Extrinsic", image_copy)
        key = cv2.waitKey(1)
        if key == ord("q"):
            logger.warn("Exiting without saving any extrinsic calibration.")
            return
        if key == ord("k") and detected_marker:
            logger.info("Exiting and saving calibration.")
            break

    extrinsic = transformation_matrix(
        rvec=rotation,
        tvec=translation,
    )

    offset_translation = np.eye(4, 4)
    offset_translation[0, 3] = options.extrinsic.offset_x
    offset_translation[1, 3] = options.extrinsic.offset_y
    extrinsic = np.dot(extrinsic, offset_translation)
    tensor = array2tensor(array=extrinsic)

    already_exists = False
    for tf in calibration.extrinsic:
        if (
            getattr(tf, "from") == options.world_id
            and getattr(tf, "to") == options.camera_id
        ):
            logger.warn("Calibration already exists, editing...")
            already_exists = True
            tf.tf.CopyFrom(tensor)
    if not already_exists:
        logger.info("Add new transformation to calibration file")
        transformation = calibration.extrinsic.add()
        transformation.tf.CopyFrom(tensor)
        setattr(transformation, "from", options.world_id)
        setattr(transformation, "to", options.camera_id)

    filename = os.path.join(options.data_dir, f"{options.camera_id}.json")
    logger.info("Writing new calibration file, file={}", filename)
    with open(file=filename, mode="w", encoding="utf-8") as file:
        content = MessageToJson(calibration, indent=2)
        file.write(content)


if __name__ == "__main__":
    main()
