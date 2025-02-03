from setuptools import find_packages, setup

package_name = 'pick_up'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sneezy',
    maintainer_email='thaeron@kth.se',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = pick_up.move_servos_publisher:main',
            'listener = pick_up.servo_pos_subscriber:main',
        ],
    },
)
