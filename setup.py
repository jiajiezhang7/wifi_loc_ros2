from setuptools import find_packages, setup
import os
from glob import glob
import sys

package_name = 'wifi_loc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'map'), glob('map/*.osm')),
    ],
    install_requires=['setuptools', 'shapely'],
    zip_safe=True,
    maintainer='jay',
    maintainer_email='jerryzhang7@126.com',
    description='WiFi-based indoor localization package',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'robot_loc = wifi_loc.robot_loc:main',
        ],
    },
    package_data={
        'wifi_loc': [
            'utils/*.py',
            'data/*.json',
        ],
    },
    # python_executable=sys.executable,  # 使用当前Python解释器路径
)
