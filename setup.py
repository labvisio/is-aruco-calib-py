from setuptools import setup, find_packages


setup(
    name='is_aruco_calib',
    version='0.0.1',
    description='',
    url='http://github.com/labvisio/is-aruco-calib',
    author='labvisio',
    license='MIT',
    packages=find_packages('.'),
    package_dir={'': '.'},
    entry_points={
        'console_scripts': [
            'is-aruco-calib-intrinsic=is_aruco_calib.calibrate.intrinsic:main',
            'is-aruco-calib-extrinsic=is_aruco_calib.calibrate.extrinsic:main',
        ],
    },
    zip_safe=False,
    install_requires=[
        "is-msgs==1.1.18",
        "is-wire==1.2.1",
        "numpy==1.24.4",
        "opencv-contrib-python==4.8.0.76",
    ],
)
