# Camera Calibration

Calibrating a camera using visual markers is a fundamental process in computer vision and AR/VR systems. It involves selecting visual markers, capturing images from different angles, identifying key points, finding point correspondences, estimating intrinsic and extrinsic camera parameters and using the calibrated camera model for tasks like 3D reconstruction and object tracking.

## *ChArUco*

ArUco markers were improved by interspersing them inside a checkerboard called ChArUco. Checkerboard corner intersections provide more stable corners because the edge location bias on one square is countered by the opposite edge orientation in the connecting square. By interspersing ArUco markers inside the checkerboard, each checkerboard corner gets a label which enables it to be used in complex calibration or pose scenarios where you cannot see all the corners of the checkerboard. [OpenCV's Tutorial abot Charuco]

![image](https://github.com/opencv/opencv_contrib/raw/master/modules/aruco/tutorials/charuco_detection/images/charucodefinition.png)

## Running

The following installation instructions are presented for the Ubuntu operating system. **Tested only on Ubuntu 22.04**.

### Virtual Environments

The `python3-venv` package is required for creating Python virtual environments. Virtual environments are useful for isolating Python projects and their dependencies. To install it in Ubuntu 22.04, run the following command:

```bash
sudo apt-get install python3-venv
```

### Cloning git repository

Now, let's clone a Git repository and create a Python virtual environment:

```bash
# Clone the Git repository
git clone https://github.com/labvisio/is-aruco-calib-py.git

# Navigate to the project directory
cd is-aruco-calib-py/

# Create a Python virtual environment with the name .venv
python3 -m venv .venv

# Activate the virtual environment
source .venv/bin/activate
```

After activating the virtual environment, your command prompt should show the environment name in parentheses, indicating that you are working within the virtual environment.

### Installing project

Now that you are in a virtual environment, install the project's dependencies using `pip`. You can proceed with the local installation of the project as mentioned:
```bash
pip3 install .
```

After completing these steps, you will have a Python virtual environment set up with the project's dependencies installed. You can run the project's code within this virtual environment, and it will not affect your global Python environment. Remember to activate the virtual environment whenever you work on this project using the source .venv/bin/activate command and deactivate it when you're done by running deactivate.


### Creating board and markers

To create markers the **is-aruco-calib-marker** script can be used. The behavior of the script can be customized by passing a JSON configuration file as the first argument. The schema for this file can be found in [is_aruco_calib/conf/options.proto]. Examples of configuration files to create a charuco board and a aruco marker can be found at [etc/conf/create-aruco.json] and [etc/conf/create-charuco.json] respectively. Usage example, run the python script passing the desired configuration file:

```shell
is-aruco-calib-marker ./etc/conf/create-aruco.json
```

### Calibrating Cameras

The script **is-aruco-calib-intrinsic** computes the intrinsic and distortion parameters using a ChArUco marker and outputs the corresponding [CameraCalibration] object as a json file. Usage example, run the python script passing the desired configuration file:

```shell
is-aruco-calib-intrinsic ./etc/conf/calibrate-aruco.json
```

The script **is-aruco-calib-extrinsic** computes the extrinsic parameters, that is, the transformation that can change poses from the camera frame to the world frame and vice-versa. The world frame of reference will coincide with the one of the aruco marker. The transformation is added to the respective [CameraCalibration] json file. Usage example, run the python script passing the desired configuration file:

```shell
is-aruco-calib-extrinsic ./etc/conf/calibrate-aruco.json
```

The behavior of the script can be customized by passing a JSON configuration file as the first argument, e.g: `is-aruco-calib-intrinsic options.json` or `is-aruco-calib-extrinsic options.json`. The schema for this configuration file can be found in [is_aruco_calib/conf/options.proto]. Example of configuration files to calibrate using a charuco board can be found at [etc/conf/calibrate-charuco.json].

<!-- Files -->
[etc/conf/create-aruco.json]: etc/conf/create-aruco.json
[etc/conf/create-charuco.json]: etc/conf/create-charuco.json
[etc/conf/calibrate-charuco.json]: etc/conf/calibrate-charuco.json
[is_aruco_calib/conf/options.proto]: is_aruco_calib/conf/options.proto


<!-- Links -->
[CameraCalibration]: https://github.com/labvisio/is-msgs/tree/master/docs#is.vision.CameraCalibration
[OpenCV's Tutorial abot Charuco]: https://github.com/opencv/opencv_contrib/blob/master/modules/aruco/tutorials/charuco_detection/charuco_detection.markdown