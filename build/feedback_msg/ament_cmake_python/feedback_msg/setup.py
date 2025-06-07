from setuptools import find_packages
from setuptools import setup

setup(
    name='feedback_msg',
    version='0.0.0',
    packages=find_packages(
        include=('feedback_msg', 'feedback_msg.*')),
)
