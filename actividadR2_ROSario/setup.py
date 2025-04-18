from setuptools import find_packages, setup

package_name = 'actividadR2_ROSario'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'localisation_ROSario = actividadR2_ROSario.localisation_ROSario:main',
            'errors_pub_ROSario = actividadR2_ROSario.errors_pub_ROSario:main'
        ],
    },
)
