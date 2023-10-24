import sys

import cv2
from is_wire.core import Logger

from is_aruco_calib.conf.options_pb2 import ArucoDictionary, CreateMarker
from is_aruco_calib.utils import load_json


def main() -> None:
    """
    The main function that creates ArUco or CharUco markers based on the options provided in a
    JSON file.

    The program reads options from a JSON file and generates ArUco or CharUco markers, depending
    on the configurations present in the file.

    Parameters
    ----------
    None (program arguments are passed via the command line or from the default options file).

    Returns
    -------
    None
    """
    logger = Logger("CreateMarker")
    filename = sys.argv[1] if len(sys.argv) > 1 else "options.json"
    options = load_json(
        filename=filename,
        schema=CreateMarker,
        logger=logger,
    )
    if options.HasField("charuco"):
        opts = options.charuco
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
        image = board.generateImage(
            (width, height),
            marginSize=int(margins),
            borderBits=opts.border_bits,
        )
        dict_name = ArucoDictionary.Name(opts.dictionary)
        cv2.imwrite(
            filename=f"charuco_{opts.n_squares_x}x{opts.n_squares_y}_{dict_name}.png",
            img=image,
        )

    elif options.HasField("aruco"):
        opts = options.aruco
        dictionary = cv2.aruco.getPredefinedDictionary(opts.dictionary)
        image = cv2.aruco.generateImageMarker(
            dictionary=dictionary,
            id=opts.marker_id,
            sidePixels=opts.marker_length,
            borderBits=opts.border_bits,
        )
        dict_name = ArucoDictionary.Name(opts.dictionary)
        cv2.imwrite(
            filename=f"aruco_{opts.marker_id}_{dict_name}.png",
            img=image,
        )
    else:
        logger.warn("Json doesnt have field `charuco` or `aruco`.")
        logger.warn("Options: {}", options)


if __name__ == "__main__":
    main()
