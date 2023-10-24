import os
import sys

import cv2
import numpy as np

from google.protobuf.json_format import MessageToJson
from is_wire.core import Logger, Channel, Subscription
from is_msgs.camera_pb2 import CameraCalibration
from is_msgs.image_pb2 import Image

from is_aruco_calib.utils import load_json, tensor2array, image2array, array2tensor
from is_aruco_calib.conf.options_pb2 import CalibrationOptions

def main() -> None:
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
            corners,
            options.extrinsic.marker_length,
            intrinsic,
            distortion,
        )
        image_copy = cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)
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
            return 0
        if key == ord("k") and detected_marker:
            break
    rmat, _ = cv2.Rodrigues(rotation)
    tvec = translation.reshape(3, 1)
    mat = np.hstack((rmat, tvec))
    mat = np.vstack((mat, np.array([[0, 0, 0, 1]])))

    offset_translation = np.eye(4, 4)
    offset_translation[0, 3] = options.extrinsic.offset
    offset_translation[1, 3] = options.extrinsic.offset

    mat = np.dot(mat, offset_translation)

    transformation = calibration.extrinsic.add()
    transformation.tf.CopyFrom(array2tensor(array=mat))
    setattr(transformation, 'from', 100 + options.extrinsic.marker_id)
    setattr(transformation, 'to', calibration.id)
    with open(
        file=os.path.join(options.data_dir, f"{options.camera_id}.json"),
        mode="w",
        encoding="utf-8",
    ) as file:
        content = MessageToJson(calibration, indent=2)
        file.write(content)


if __name__ == "__main__":
    main()