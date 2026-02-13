import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ackermann_chassis_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ackermann_chassis_driver']),
        ('share/ackermann_chassis_driver', ['package.xml']),
        (os.path.join('share', 'ackermann_chassis_driver', 'launch'), glob('launch/*.py')),
        (os.path.join('share', 'ackermann_chassis_driver', 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='beck',
    maintainer_email='beck@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'chassis_driver_node = ackermann_chassis_driver.chassis_driver_node:main',
        ],
    },
)
