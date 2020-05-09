# -*- coding: utf-8 -*-
from __future__ import absolute_import

from rospy_message_converter import json_message_converter
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
from abc import ABCMeta

import inject
import rospy
from std_msgs.msg import Bool
import json

from .util import lookup_object, extract_values, populate_instance
from .mqtt_client import createMqttClient, customCallback


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
    _extract_private_path = inject.attr('mqtt_private_path_extractor')


class RosToMqttBridge(Bridge):
    u""" Bridge from ROS topic to MQTT

    :param str topic_from: incoming ROS topic path
    :param str topic_to: outgoing MQTT topic path
    :param class msg_type: subclass of ROS Message
    :param (float|None) frequency: publish frequency
    """

    def __init__(self, topic_from, topic_to, msg_type, frequency=None):
        self._topic_from = topic_from
        self._topic_to = self._extract_private_path(topic_to)

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
        self._mqtt_client.publish(topic=self._topic_to, payload=payload, QoS=1)

    def disconnect():
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
        self._topic_from = self._extract_private_path(topic_from)
        print("!!!!!%s resolved to %s" % (topic_from, self._topic_from))
        self._topic_to = topic_to
        self._msg_type = msg_type
        self._queue_size = queue_size
        self._last_published = rospy.get_time()
        self._interval = None if frequency is None else 1.0 / frequency
        self._mqtt_client = createMqttClient()
        self._mqtt_client.subscribe(topic_from, 1, self._callback_mqtt)
        self._mqtt_client.connect()
        self._publisher = rospy.Publisher(
            self._topic_to, self._msg_type, queue_size=self._queue_size)

    def disconnect():
        self._mqtt_client.disconnect()

    def _callback_mqtt(self, client, userdata, mqtt_msg):
        u""" callback from MQTT

        :param mqtt.Client client: MQTT client used in connection
        :param userdata: user defined data
        :param mqtt.MQTTMessage mqtt_msg: MQTT message
        """

        print("got %s, expected %s: running: %r" % ( mqtt_msg.topic, self._topic_from, mqtt_msg.topic == self._topic_from))
        if mqtt_msg.topic != self._topic_from:
            return

        rospy.loginfo("MQTT received from {}".format(mqtt_msg.topic))
        now = rospy.get_time()

        if self._interval is None or now - self._last_published >= self._interval:
            try:
                ros_msg = self._create_ros_message(mqtt_msg)
                print("publisher of type %s" % (self._msg_type))
                print("publishes to topic %s" % (self._topic_to))
                print("got from topic %s" % (self._topic_from))
                self._publisher.publish(ros_msg)
                self._last_published = now
            except Exception as e:
                rospy.logerr(e)

    def _create_ros_message(self, mqtt_msg):
        u""" create ROS message from MQTT payload

        :param mqtt.Message mqtt_msg: MQTT Message
        :return rospy.Message: ROS Message
        """
        print("got msg %s" % (mqtt_msg.payload))
        ros_message = Bool(data=True if mqtt_msg.payload == "true" else False)
        return ros_message
        # msg_dict = json_message_converter.convert_json_to_ros_message(ros_message)
        # return populate_instance(msg_dict, self._msg_type()) #qué putas!!!


__all__ = ['register_bridge_factory', 'create_bridge', 'Bridge',
           'RosToMqttBridge', 'MqttToRosBridge']
