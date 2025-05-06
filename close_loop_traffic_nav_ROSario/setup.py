from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'close_loop_traffic_nav_ROSario'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), # Launch File
        glob(os.path.join('launch', '*launch.[pxy][yma]*'))), 
        (os.path.join('share', package_name, 'config'), # Parameter File
        glob(os.path.join('config', '*.[yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oscar_ot09',
    maintainer_email='oscar_ot09@outlook.es',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_ROSario = close_loop_traffic_nav_ROSario.controller_ROSario:main',
            'localisation_ROSario = close_loop_traffic_nav_ROSario.localisation_ROSario:main',
            'path_generator_ROSario = close_loop_traffic_nav_ROSario.path_generator_ROSario:main',
            'pose_plotter = close_loop_traffic_nav_ROSario.pose_plotter:main',
            'colorIdentificator = close_loop_traffic_nav_ROSario.colorIdentificator:main'
        ],
    },
)
