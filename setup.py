#!/usr/bin/python3
from setuptools import setup
from distutils.core import Extension
import sysconfig
import platform

# Grab the package version from version.py
about = {}
with open("pychronos/about.py") as fp:
    exec(fp.read(), about)

DESCRIPTION=("Python bindings for the Chronos High Speed Camera")

# Tune CFLAGS for ARMv7 architecture builds.
extra_cflags = sysconfig.get_config_var('CFLAGS').split()
extra_ldflags = sysconfig.get_config_var('LDFLAGS').split()
if (platform.machine() == 'armv7l'):
    extra_cflags += ["-mfloat-abi=softfp", "-mcpu=cortex-a8", "-mfpu=neon"]
    extra_ldflags += ["-mfloat-abi=softfp", "-mcpu=cortex-a8", "-mfpu=neon"]

libpychronos = Extension('libpychronos',
                        depends = [
                            'pychronos/lib/fpga.h',
                            'pychronos/lib/pychronos.h'],
                        sources = [
                            'pychronos/lib/module.c',
                            'pychronos/lib/fpgamap.c',
                            'pychronos/lib/frame.c',
                            'pychronos/lib/pwm.c'],
                        extra_compile_args=extra_cflags,
                        extra_link_args=extra_ldflags)

setup (name='PyChronos',
       version=about['__version__'],
       description=DESCRIPTION,
       license='GPL',
       url = 'https://github.com/krontech/pychronos',
       author = about['__author__'],
       author_email = 'oskirby@gmail.com',
       entry_points={
           'console_scripts': [ 'camControl=camControl.__main__:main' ]
       },
       packages=[
           'camControl',
           'pychronos',
           'pychronos/regmaps',
           'pychronos/sensors'
       ],
       ext_modules = [libpychronos])
