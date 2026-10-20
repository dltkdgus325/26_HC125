import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'boat_autonomy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*.html')),
        (os.path.join('share', package_name, 'web', 'maps'),
            glob('web/maps/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rodin',
    maintainer_email='rodin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'los_controller = boat_autonomy.los_controller:main',
            'boat_comm_node = boat_autonomy.stm_bridge:main',
            'gps_logger = boat_autonomy.gps_logger:main',
            'camera_recorder = boat_autonomy.camera_recorder:main',
            'camera_publisher = boat_autonomy.camera_publisher:main',
            'mission_recorder = boat_autonomy.mission_recorder:main',
            'trash_detection = boat_autonomy.trash_detection:main',
            'trash_controller = boat_autonomy.trash_controller:main',
            'autonomy_dashboard = boat_autonomy.autonomy_dashboard:main',
        ],
    },
)
