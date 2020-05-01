# Check to see if AWS Device SDK for Python is already installed, install if not
if ! python -c "import AWSIoTPythonSDK" &> /dev/null; then
  printf "\nInstalling AWS SDK...\n"
  pushd aws-iot-device-sdk-python
  python setup.py install
  result=$?
  popd
  if [ $result -ne 0 ]; then
    printf "\nERROR: Failed to install SDK.\n"
    exit $result
  fi
fi