from setuptools import find_packages
from setuptools import setup

setup(
    name='drone_assembly_cell',
    version='0.1.0',
    packages=find_packages(
        include=('drone_assembly_cell', 'drone_assembly_cell.*')),
)
