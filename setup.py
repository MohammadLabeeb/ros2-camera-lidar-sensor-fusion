from setuptools import setup
import os
from glob import glob

package_name = 'environmental_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='labeeb',
    maintainer_email='lab1526s@hs-coburg.de',
    description='A package for sensor data fusion using ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'environmental_model = environmental_model.environmental_model:main'
        ],
    },
)
