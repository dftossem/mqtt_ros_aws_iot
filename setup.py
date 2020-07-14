import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="mqtt_ros_aws_iot",
    version="0.0.1",
    author="David Tosse",
    author_email="dftossem@unal.edu.co",
    description="Connection to AWS IoT from ROS through MQTT",
    long_description=long_description,
    url="https://github.com/dftossem/mqtt_ros_aws_iot",
    packages=['mqtt_bridge'],
    package_dir={'': 'src'},
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: Ubuntu Bionic Beaver",
    ],
    python_requires='>=2.7',
    install_requires=['bson>=0.5.2',
                      'inject==3.5.1',
                      'pymongo>=3.10.1',
                      'msgpack-python>=0.4.8',
                      'AWSIoTPythonSDK>=1.4.8'],
)
