import sys

import cv2
import numpy as np
import numpy.typing as npt

from is_wire.core import Logger

from is_aruco_calib.conf.options_pb2 import (
    CreateMarker,
    CreateArucoOptions,
    CreateCharucoOptions,
    ArucoDictionary,
)
from is_aruco_calib.utils import load_json

def main():
    logger = Logger("CreateMarker")
    filename = sys.argv[1] if len(sys.argv) > 1 else "options.json"
    options = load_json(
        filename=filename,
        schema=CreateMarker,
        logger=logger,
    )
    if options.HasField("charuco"):
        opts =  options.charuco
        dictionary = cv2.aruco.getPredefinedDictionary(opts.dictionary)
        margins = opts.square_length - opts.marker_length
        width = int((opts.n_squares_x * opts.square_length) + (2 * margins))
        height = int((opts.n_squares_y * opts.square_length) + (2 * margins))

        board = cv2.aruco.CharucoBoard(
            size=(opts.n_squares_x, opts.n_squares_y),
            squareLength=opts.square_length,
            markerLength=opts.marker_length,
            dictionary=dictionary,
        )
        image = board.generateImage((width, height), marginSize=int(margins), borderBits=opts.border_bits)
        cv2.imwrite(
            filename=f"charuco_{opts.n_squares_x}x{opts.n_squares_y}_{ArucoDictionary.Name(opts.dictionary)}.png",
            img=image,
        )
        
    elif options.HasField("aruco"):
        opts =  options.aruco
        dictionary = cv2.aruco.getPredefinedDictionary(opts.dictionary)
        image = cv2.aruco.generateImageMarker(
            dictionary=dictionary,
            id=opts.marker_id,
            sidePixels=opts.marker_length,
            borderBits=opts.border_bits,
        )
        cv2.imwrite(
            filename=f"aruco_{opts.marker_id}_{ArucoDictionary.Name(opts.dictionary)}.png",
            img=image,
        ) 
    else:
        logger.critical("Could not generate aruco or charuco...")

if __name__ == "__main__":
    main()