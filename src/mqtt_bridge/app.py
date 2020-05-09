# -*- coding: utf-8 -*-
from __future__ import absolute_import

import time
import json
import rospy
import inject

from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

from .mqtt_client import create_private_path_extractor
from .bridge import create_bridge
from .util import lookup_object

def create_config(mqtt_client, mqtt_private_path):
    private_path_extractor = create_private_path_extractor(mqtt_private_path)
    def config(binder):
        binder.bind(AWSIoTMQTTClient, mqtt_client)
        binder.bind('mqtt_private_path_extractor', private_path_extractor)
    return config

def mqtt_bridge_node():
    # init node
    rospy.init_node('mqtt_bridge_node')

    # load parameters
    params = rospy.get_param('~', {})
    mqtt_params = params.get('mqtt')
    bridge_params = params.pop('bridge', [])
    mqtt_private_path = mqtt_params.get('private_path', '')

    # create mqtt client
    mqtt_client_factory_name = rospy.get_param(
        '~mqtt_client_factory', '.mqtt_client:default_mqtt_client_factory')
    mqtt_client_factory = lookup_object(mqtt_client_factory_name)
    mqtt_client = mqtt_client_factory(params)

    # dependency injection
    config = create_config(
        mqtt_client, mqtt_private_path)
    inject.configure(config)
    
    # configure bridges
    bridges = []
    for bridge_args in bridge_params:
        bridges.append(create_bridge(**bridge_args))

    def disconnect():
        mqtt_client.disconnect()
        for bridge in bridges:
            bridge.disconnect()

    rospy.on_shutdown(disconnect)

    # Connect and subscribe to AWS IoT
    mqtt_client.connect()

    rospy.spin()

__all__ = ['mqtt_bridge_node']
