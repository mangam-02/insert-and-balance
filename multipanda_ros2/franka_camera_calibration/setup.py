import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'franka_camera_calibration'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paul Kroeger',
    maintainer_email='paulkroeger10@gmail.com',
    description='Wrist-camera (eye-in-hand) extrinsic calibration for the Franka Panda.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_charuco_board = franka_camera_calibration.generate_charuco_board:main',
            'record_poses = franka_camera_calibration.record_poses:main',
            'capture_samples = franka_camera_calibration.capture_samples:main',
            'calibrate_from_captures = franka_camera_calibration.calibrate_from_captures:main',
            'validate_calibration = franka_camera_calibration.validate_calibration:main',
            'calibrate_wrist_camera = franka_camera_calibration.calibrate_wrist_camera:main',
        ],
    },
)
