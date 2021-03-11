import numpy
from setuptools import setup
from distutils.extension import Extension
from Cython.Build import cythonize


extensions = [
    Extension("chain.joint_chain", ['chain/joint_chain.pyx'],
              include_dirs=[numpy.get_include(), 'chain/include/',  'eigen/']),
    ]

setup(
    name='chain',
    ext_modules=cythonize(extensions),
    install_requires=['numpy']
    )
