from setuptools import find_packages, setup

package_name = 'ball_balance'

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
    description='Challenge 2 — balance a table-tennis ball on a TCP-mounted plate.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'ball_tracker = ball_balance.ball_tracker:main',
            # 'balance_controller = ball_balance.balance_controller:main',
        ],
    },
)
