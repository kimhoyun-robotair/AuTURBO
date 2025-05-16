from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aerial_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimhoyun',
    maintainer_email='suberkut76@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "px4_odom = aerial_nav2.px4_odom:main",
            "map_frame_publisher = aerial_nav2.map_frame_publisher:main",
            "odom_to_base_link = aerial_nav2.odom_to_base_link:main",
        ],
    },
)
