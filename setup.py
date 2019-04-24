#!/usr/bin/python3
from distutils.core import setup, Extension
import sysconfig

VERSION='0.3.1'
DESCRIPTION=("Python bindings for the Chronos High Speed Camera")

extra_cflags = sysconfig.get_config_var('CFLAGS').split()
extra_cflags += ["-mfloat-abi=softfp", "-mcpu=cortex-a8", "-mfpu=neon"]
extra_ldflags = sysconfig.get_config_var('LDFLAGS').split()
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
       version=VERSION,
       description=DESCRIPTION,
       license='GPL',
       url = 'https://github.com/krontech/pychronos',
       author = 'Owen Kirby',
       author_email = 'oskirby@gmail.com',
       provides=['pychronos'],
       packages=['pychronos', 'pychronos/regmaps', 'pychronos/sensors'],
       ext_modules = [libpychronos])
