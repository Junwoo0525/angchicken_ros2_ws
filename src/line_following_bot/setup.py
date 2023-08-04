from setuptools import setup

package_name = 'line_following_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='choi',
    maintainer_email='jwchoi0017@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_acquisition_node = line_following_bot.image_acquisition_node:main',
            'lane_detection_node = line_following_bot.lane_detection_node:main',
        ],
    },
)
