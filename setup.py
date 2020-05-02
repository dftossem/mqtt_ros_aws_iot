# -*- coding: utf-8 -*-

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['mqtt_ros_aws_iot'],
    package_dir={'': 'src'},
    install_requires=['bson', 'inject', 'msgpack-python', 'pymongo', 'AWSIoTPythonSDK']
)

setup(**setup_args)
