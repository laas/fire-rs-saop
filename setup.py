#!/usr/bin/env python3
from setuptools import find_packages, setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

setup(
    name="Fire RS",
    version="0.1",
    description="FireRS situation assessment and observation planning (SAOP) software",
    url="http://fire-rs.com",
    packages=find_packages(),
    license="GPLv3+",
    install_requires=['affine', 'GDAL', 'numpy', 'pandas', 'pytz', 'matplotlib', 'sklearn'],
    ext_modules=[
        Extension("fire_rs.firemodel.rothermel", sources=["fire_rs/firemodel/rothermel.pyx"]),
        Extension("fire_rs.firemodel.environment", sources=["fire_rs/firemodel/environment.pyx"]),
        Extension("fire_rs.firemodel.fireshapes", sources=["fire_rs/firemodel/fireshapes.pyx"]),
        # Extension("fire_rs.firemodel.propagation", sources=["fire_rs/firemodel/propagation.pyx"]),
    ],
    cmdclass={'build_ext': build_ext}
)
