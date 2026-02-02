from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ttt_control'

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
    description='Tic-Tac-Toe Control Node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'picknplace_node = ttt_control.picknplace_node:main',
        'pnp_test_node = ttt_control.pnp_test_node:main',
        'simulate2machine = ttt_control.simulate2machine:main',
        'pnp_calibrate = ttt_control.pnp_calibrate:main',
        
        ],
    },
)
