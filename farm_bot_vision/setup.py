from setuptools import setup
import os
from glob import glob

package_name = 'farm_bot_vision'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        'pyserial',
        'ultralytics',
        'gpiozero',
        'Pillow',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='mario',
    maintainer_email='mario@todo.todo',
    description='Farm bot vision and control system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_mode = farm_bot_vision.control_mode:main',
            'navigation_camera = farm_bot_vision.navigation_camera:main',
            'crop_camera = farm_bot_vision.crop_camera:main',
            'receiver_node = farm_bot_vision.receiver_node:main',
            'serial_communication = farm_bot_vision.serial_communication:main',
            'crop_navigator = farm_bot_vision.crop_navigator:main',
            'lidar_node = farm_bot_vision.lidar_node:main',
            'ultrasonic_node = farm_bot_vision.ultrasonic_node:main',
        ],
    },
)