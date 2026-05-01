from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'human_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='technomant',
    maintainer_email='kadyrgulov.01@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'thermal_detection_node = human_detector.thermal_detection_node:main',
            'radar_detection_node = human_detector.radar_detection_node:main',
            'fusion_node = human_detector.fusion_node:main',
        ],
    },
)
