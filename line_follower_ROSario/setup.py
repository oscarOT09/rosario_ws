from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'line_follower_ROSario'

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
            'controller_ROSario = line_follower_ROSario.controller_ROSario:main',
            'lineDetector_ROSario = line_follower_ROSario.lineDetector_ROSario:main',
            'localisation_ROSario = line_follower_ROSario.localisation_ROSario:main',
            'trafficLightDetector = line_follower_ROSario.trafficLightDetector_ROSario:main',
            'camera_recorder = line_follower_ROSario.camera_recorder:main',
        ],
    },
)
