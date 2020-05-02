# MQTT protocol with ROS and AWS IoT compatible

##

This package is an adaptation from [mqtt_bridge repository](https://github.com/groove-x/mqtt_bridge) and uses AWSIoTPythonSDK instead of paho-mqtt.

## Download

Clone the repository in your ROS workspace

    cd ~/catkin_ws/src
    git clone https://github.com/dftossem/mqtt_ros_aws_iot.git

## Install requirements

Install python requirements.

    cd mqtt_ros_aws_iot
    pip install -r requirements.txt

## Change connection parameters

In the *config/params.yaml* file change host (endpoint) and certificates. If you have installed the [AWS CLI](https://docs.aws.amazon.com/cli/latest/userguide/install-cliv2.html) then you can find the host variable by typing in a terminal:

    aws iot describe-endpoint --endpoint-type iot:Data-ATS

## Test

### Build the package

Use catkin to build your ROS workspace

    cd ~/catkin_ws
    catkin_make

### Update source

    source ~/catkin_ws/devel/setup.bash

### Launch

    roslaunch mqtt_ros_aws_iot connect.launch

### Verify on AWS IoT Core page

Login to your AWS account and go to the AWS IoT Core page, then in the **Test** option set *sdk/test/Python* as Subscription topic and click the **Subscribe to topic** button.

### Local topic publish

In a new terminal publish the ROS topic

    rostopic pub /ping std_msgs/Bool "data: false"

You should see the local topic payload data on the AWS IoT page just like this:

```javascript
{
  "data": false
}
```
