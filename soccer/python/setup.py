
# this setup.py file builds the python extension module for robocup
# it may be deleted in the future in favor of just having
# scons do the compiling of stuff

from distutils.core import setup, Extension
import os

module1 = Extension('robocup',
	sources = ['robocup.cpp'],
	libraries=['boost_python-py33'])

setup(name='robocup-soccer',
	ext_modules=[module1])
