from setuptools import setup
import os
from glob import glob

package_name = 'stewart_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sorak',
    maintainer_email='mcdevittgpt@gmail.com',
    description='Laptop/Pi bring-up for the Stewart platform: global speed caps, jog CLI, launch files.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stewart_control_node = stewart_bringup.stewart_control_node:main',
        ],
    },
)
