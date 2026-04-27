from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'embedded_ros_marvin'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ericb',
    maintainer_email='ericbi@umich.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "dual_odrive_controller = embedded_ros_marvin.odrive_two_motors:main",
            "LED_subscriber = embedded_ros_marvin.led_subscriber:main",
            "recovery_executable = embedded_ros_marvin.recovery_executable:main",
            "serial_estop_monitor = embedded_ros_marvin.serial_estop_monitor:run_estop_monitor",
        ],
    },
)
