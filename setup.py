from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'Embedded_ROS_Marvin'

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
            "odrive_controller = arv_embedded.odrive_sub_pub:main",
            "dual_odrive_controller = arv_embedded.odrive_two_motors:main",
            "LED_subscriber = arv_embedded.led_subscriber:main",
            "enc_odom_publisher = arv_embedded.enc_odom_publisher:main"
        ],
    },
)
