# -*- coding: utf-8 -*-
from __future__ import absolute_import

from rospy_message_converter import json_message_converter, message_converter
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from abc import ABCMeta

from .util import lookup_object

import inject
import rospy
import json
import re

def create_bridge(factory, msg_type, topic_from, topic_to):
    u""" bridge generator function

    :param (str|class) factory: Bridge class
    :param (str|class) msg_type: ROS message type
    :param str topic_from: incoming topic path
    :param str topic_to: outgoing topic path
    :param (float|None) frequency: publish frequency
    :return Bridge: bridge object
    """
    if isinstance(factory, basestring):
        factory = lookup_object(factory)
    if not issubclass(factory, Bridge):
        raise ValueError("factory should be Bridge subclass")
    if isinstance(msg_type, basestring):
        msg_type = lookup_object(msg_type)
    if not issubclass(msg_type, rospy.Message):
        raise TypeError(
            "msg_type should be rospy.Message instance or its string"
            "reprensentation")
    return factory(
        topic_from=topic_from, topic_to=topic_to, msg_type=msg_type)


class Bridge(object):
    u""" Bridge base class
    :param mqtt.Client _mqtt_client: MQTT client
    """
    __metaclass__ = ABCMeta
    _mqtt_client = inject.attr(AWSIoTMQTTClient)

class RosToMqttBridge(Bridge):
    u""" Bridge from ROS topic to MQTT

    :param str topic_from: incoming ROS topic path
    :param str topic_to: outgoing MQTT topic path
    :param class msg_type: subclass of ROS Message
    :param (float|None) frequency: publish frequency
    """

    def __init__(self, topic_from, topic_to, msg_type, frequency=None):
        self._topic_from = topic_from
        self._topic_to = topic_to

        self._last_published = rospy.get_time()
        self._interval = 0 if frequency is None else 1.0 / frequency
        rospy.Subscriber(topic_from, msg_type, self._callback_ros)

    def _callback_ros(self, msg):
        rospy.logdebug("ROS received from {}".format(self._topic_from))
        now = rospy.get_time()
        if now - self._last_published >= self._interval:
            self._publish(msg)
            self._last_published = now

    def _publish(self, msg):
        payload = json_message_converter.convert_ros_message_to_json(msg)
        self._mqtt_client.publish(topic=self._topic_to, payload=payload, QoS=0)

    def disconnect(self):
        pass
class MqttToRosBridge(Bridge):
    u""" Bridge from MQTT to ROS topic

    :param str topic_from: incoming MQTT topic path
    :param str topic_to: outgoing ROS topic path
    :param class msg_type: subclass of ROS Message
    :param (float|None) frequency: publish frequency
    :param int queue_size: ROS publisher's queue size
    """
    def __init__(self, topic_from, topic_to, msg_type, frequency=None,
                 queue_size=10):
        self._topic_from = topic_from
        self._topic_to = topic_to
        self._msg_type = msg_type
        self._queue_size = queue_size
        self._last_published = rospy.get_time()
        self._interval = None if frequency is None else 1.0 / frequency
        ans = self._mqtt_client.subscribe(topic_from, 0, self._callback_mqtt)
        self._publisher = rospy.Publisher(
            self._topic_to, self._msg_type, queue_size=self._queue_size)

    def _callback_mqtt(self, client, userdata, mqtt_msg):
        u""" callback from MQTT

        :param mqtt.Client client: MQTT client used in connection
        :param userdata: user defined data
        :param mqtt.MQTTMessage mqtt_msg: MQTT message
        """
        rospy.logdebug("MQTT received from {}".format(mqtt_msg.topic))
        now = rospy.get_time()
        
        if self._interval is None or now - self._last_published >= self._interval:
            try:
                ros_msg = self._create_ros_message(mqtt_msg)
                self._publisher.publish(ros_msg)
                self._last_published = now
            except Exception as e:
                rospy.logerr(e)

    def _create_ros_message(self, mqtt_msg):
        u""" create ROS message from MQTT payload

        :param mqtt.Message mqtt_msg: MQTT Message
        :return rospy.Message: ROS Message
        """
        cnvrt_type = self._get_rosType(str(self._msg_type))
        if(cnvrt_type == 'std_msgs/Bool'):
            payload = Bool(data=True if self._get_boolVal(mqtt_msg.payload) else False)
            ros_message = payload
        elif(cnvrt_type == 'geometry_msgs/Twist'):
            message = json.dumps(json.loads(mqtt_msg.payload))
            msg = json_message_converter.convert_json_to_ros_message('geometry_msgs/Twist', message)
            ros_message = msg
        else:
            ros_message = message_converter.convert_dictionary_to_ros_message(cnvrt_type, {'data':  mqtt_msg.payload})
        return ros_message

    def _get_rosType(self, msg_type):
        parts = msg_type.split('_')
        pckg = parts[0].split("'")[1]
        msgType = parts[2].split(".")[0]
        return pckg + '_msgs/' + msgType

    def _get_boolVal(self, payload):
        return payload.lower() in ("yes", "true", "1")

__all__ = ['register_bridge_factory', 'create_bridge', 'Bridge',
           'RosToMqttBridge', 'MqttToRosBridge']
