#!/usr/bin/env python
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  ##  don't do this unless you want a globally visible script
  #scripts=['bin/sickldmrs.py'], 
  packages=['sick_ldmrs'],
  package_dir={'': 'src'}
)

setup(**d)
