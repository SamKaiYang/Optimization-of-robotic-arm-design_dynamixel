#!/usr/bin/env python3
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
from sympy import E

d = generate_distutils_setup()
d['packages'] = ['dynamics']
d['package_dir'] = {'': 'dynamics'}

setup(**d)

e = generate_distutils_setup()
e['packages'] = ['drl']
e['package_dir'] = {'': 'drl'}

setup(**e)
