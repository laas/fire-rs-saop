#!/usr/bin/env python3
from setuptools import find_packages, setup

setup(
    name="Fire RS",
    version="0.1",
    description="FireRS situation assessment and observation planning (SAOP) software",
    url="http://fire-rs.com",
    packages=find_packages(),
    license="GPLv3+",
    install_requires=['affine', 'GDAL', 'numpy', 'pytz']
)
