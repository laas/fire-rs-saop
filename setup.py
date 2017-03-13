#!/usr/bin/env python3
from distutils.core import setup

setup(
    name="FireRS",
    version="0.1",
    description="FireRS software",
    url="http://fire-rs.com",
    packages=['fire_rs',],
    license="GPLv3+",
    install_requires=['pandas', 'numpy']
)
