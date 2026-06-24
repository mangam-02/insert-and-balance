from setuptools import find_packages, setup

package_name = 'insertion'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Timo Matuszewski',
    maintainer_email='timo_matuszewski@online.de',
    description='Challenge 1 — compliant peg-in-hole insertion (vision + force).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'insertion_fsm = insertion.insertion_fsm:main',
            # 'socket_detection = insertion.socket_detection:main',
        ],
    },
)
