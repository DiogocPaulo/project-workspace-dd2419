from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'detection'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append((os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))))
data_files.append((os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sneezy',
    maintainer_email='thaeron@kth.se',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection = detection.detection:main',
            'detection_node = detection.detection_node:main',
            'object_filter_node = detection.object_filter_node:main',
        ],
    },
)
