from setuptools import setup

package_name = 'peg_in_hole_pipeline'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/peg_in_hole_pipeline.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minga-09',
    maintainer_email='paulkroeger10@gmail.com',
    description='State-machine peg grasp + insertion pipeline (FoundationPose + MoveIt + franka_gripper).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pipeline = peg_in_hole_pipeline.pipeline_node:main',
        ],
    },
)
