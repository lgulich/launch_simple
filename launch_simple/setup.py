import glob
import os

from setuptools import setup

PACKAGE_NAME = 'launch_simple'

setup(
    name=PACKAGE_NAME,
    version='0.0.1',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'launch'), glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lionel Gulich',
    maintainer_email='lionel.gulich@gmail.com',
    description='A tool to simplify ROS2 launch files',
    license='TBD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
