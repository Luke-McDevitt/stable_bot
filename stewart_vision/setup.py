from setuptools import setup
import os
from glob import glob

package_name = 'stewart_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml.template')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sorak',
    maintainer_email='mcdevittgpt@gmail.com',
    description='Vision pipeline for Stable-Bot: OAK-D Pro AF, ArUco ring, ball KF, ref generator.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oak_driver_node = stewart_vision.oak_driver_node:main',
            'platform_pose_node = stewart_vision.platform_pose_node:main',
            'ball_localizer_node = stewart_vision.ball_localizer_node:main',
            'ball_kf_node = stewart_vision.ball_kf_node:main',
            'ref_generator_node = stewart_vision.ref_generator_node:main',
        ],
    },
)
