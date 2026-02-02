from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ttt_cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson-nx@example.com',
    description='Tic-Tac-Toe Vision Processing Node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'camera_tuning	= ttt_cv.camera_tuning:main',
        'image_debug	= ttt_cv.image_debug:main',
        'vision_node = ttt_cv.vision_node:main',
        'vision_client_test = ttt_cv.vision_client_test:main',
        'camera_arm_node = ttt_cv.camera_arm_node:main',
        
        ],
    },
)
