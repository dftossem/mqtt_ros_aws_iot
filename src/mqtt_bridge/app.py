# -*- coding: utf-8 -*-
from __future__ import absolute_import

import inject
import rospy
import time
import json

from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

from .bridge import create_bridge
from .util import lookup_object

def mqtt_bridge_node():
    # init node
    rospy.init_node('mqtt_bridge_node')

    # load parameters
    params = rospy.get_param('~', {})
    bridge_params = params.get('bridge', [])

    # create mqtt client
    mqtt_client_factory_name = rospy.get_param(
        '~mqtt_client_factory', '.mqtt_client:createMqttClient')
    mqtt_client_factory = lookup_object(mqtt_client_factory_name)
    mqtt_client = mqtt_client_factory(params)

    # dependency injection
    config = create_config(mqtt_client)
    inject.configure(config)
    
    # configure bridges, one per factory
    bridges = []
    for bridge_args in bridge_params:
        bridges.append(create_bridge(**bridge_args))
    
    rospy.on_shutdown(mqtt_client.disconnect)

    # Connect and subscribe to AWS IoT
    mqtt_client.connect()

    rospy.spin()

def create_config(mqtt_client):
    def config(binder):
        binder.bind(AWSIoTMQTTClient, mqtt_client)
    return config

__all__ = ['mqtt_bridge_node']