# -*- coding: utf-8 -*-
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

import ssl
import time
import json
import rospy
import logging

AllowedActions = ['both', 'publish', 'subscribe']

def createMqttClient(params):
    u""" MQTT Client factory

    :param dict param: configuration parameters
    :return mqtt.Client: MQTT Client
    """
    # create client
    top_params = params.get('topic')
    tls_params = params.get('tls', {})
    mqtt_params = params.get('mqtt', {})
    conn_params = mqtt_params.get('connection', {})
    client_params = mqtt_params.get('client', {})

    host = conn_params['host']
    mode = conn_params['mode']
    webSckt = client_params['websocket']
    tls_cert = tls_params['certKey']
    tls_privKey = tls_params['privKey']
    tls_ca = tls_params['rootCA']
    port = conn_params['port']

    if mode not in AllowedActions:
        rospy.logerr("Unknown --mode option %s. Must be one of %s" %
                     (mode, str(AllowedActions)))
        exit(2)

    if webSckt and tls_cert and tls_privKey:
        rospy.logerr(
            "X.509 cert authentication and WebSocket are mutual exclusive. Please pick one.")
        exit(2)

    if not webSckt and (not tls_cert or not tls_privKey):
        rospy.logerr("Missing credentials for authentication.")
        exit(2)

    # Port defaults
    if webSckt and not port:  # When no port override for WebSocket, default to 443
        port = 443
    if not webSckt and not port:  # When no port override for non-WebSocket, default to 8883
        port = 8883

    # # Configure logging, commented = off
    # logger = logging.getLogger("AWSIoTPythonSDK.core")
    # logger.setLevel(logging.DEBUG)
    # streamHandler = logging.StreamHandler()
    # formatter = logging.Formatter(
    #     '%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    # streamHandler.setFormatter(formatter)
    # logger.addHandler(streamHandler)

    # Init AWSIoTMQTTClient
    myAWSIoTMQTTClient = None
    if webSckt:
        myAWSIoTMQTTClient = AWSIoTMQTTClient(
            client_params['clientId'], useWebsocket=True)
        myAWSIoTMQTTClient.configureEndpoint(host, port)
        myAWSIoTMQTTClient.configureCredentials(tls_ca)
    else:
        myAWSIoTMQTTClient = AWSIoTMQTTClient(client_params['clientId'])
        myAWSIoTMQTTClient.configureEndpoint(host, port)
        myAWSIoTMQTTClient.configureCredentials(tls_ca, tls_privKey, tls_cert)

    # AWSIoTMQTTClient connection configuration
    myAWSIoTMQTTClient.configureAutoReconnectBackoffTime(1, 32, 20)
    # Infinite offline Publish queueing
    myAWSIoTMQTTClient.configureOfflinePublishQueueing(-1)
    myAWSIoTMQTTClient.configureDrainingFrequency(2)  # Draining: 2 Hz
    myAWSIoTMQTTClient.configureConnectDisconnectTimeout(20)  # 10 sec
    myAWSIoTMQTTClient.configureMQTTOperationTimeout(5)  # 5 sec

    return myAWSIoTMQTTClient

__all__ = ['createMqttClient']
