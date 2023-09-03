import os
from glob import glob
from setuptools import setup
package_name = 'pump_track'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='choi',
    maintainer_email='jwchoi0017@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'pump_pub = pump_track.pump_publisher:main',
                'pump_pub_2 = pump_track.pump_publisher_ver2:main',
                'diffbot_cont = pump_track.diffbot_control:main',
                'army_detect = pump_track.army_detection:main',
                'usb_cam = pump_track.usb_camera_node:main',
                'track_pub_node = pump_track.track_pub_node:main',
        ],
    },
)
