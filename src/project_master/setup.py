from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'project_master'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append((os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='thaeron@kth.se',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'project_master = project_master.project_master:main',
            'explore_master = project_master.explore_master:main',
            'collect_master = project_master.collect_master:main',
        ],
    },
)
