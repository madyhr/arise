from setuptools import find_packages
from setuptools import setup

setup(
    name='arise',
    version='0.0.0',
    packages=find_packages(
        include=('arise', 'arise.*')),
)
