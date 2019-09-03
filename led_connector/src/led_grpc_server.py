#!/usr/bin/env python

################################################################################
#
#   Copyright (c) 2017, Fraunhofer Institut IFAM,
#                       Ottenbecker Damm 12, D 21684 Stade, Germany.
#   All rights reserved.
#
#   Redistribution in source and binary forms, with or without
#   modification, are not permitted.
#
#   The right to use is restricted to the Airbus research and development
#   department for the use cases in the 'factory of the future'.
#   Order number: 8206140758
#
#   The operation of this software is only allowed on devices that are
#   certificated and prepared by the copyright owner.
#
#   Author: Julian Bonas
#   Email:  julian.bonas@ifam.fraunhofer.de
#
################################################################################

import os
import socket
import json
import time
import logging

import grpc
import led_connector_pb2_grpc as led_serv

from concurrent import futures

if 'DEBUG' in os.environ:
    DEBUG = os.environ['DEBUG']
else:
    DEBUG = False

class LEDStateIndicator(led_serv.LEDStateIndicatorServicer):

    def __init__(self, led_ip):
        self._led_ip = led_ip

    def setLEDState(self, request, context):
        msg = json.dumps({'color': request.color,
                          'frequency': request.frequency})

        self.send(msg)

        return request

    def setLEDBlinkFreqency(self, request, context):
        msg = json.dumps({'frequency': request.frequency})
        self.send(msg)

        return request

    def setLEDConfig(self, request, context):
        msg = json.dumps({'static_ip': request.ip,
                          'mac_address': request.mac,
                          'led_count': request.led_count})

    def resetLED(self, request, context):
        msg = json.dumps({'reset': 1})
        self.send(msg)

        return request

    def send(self, msg):

        if DEBUG:
            logging.debug(msg)
            return

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((self._led_ip, 80))
            s.sendall(msg.encode())
            s.close()

        except OSError as e:
            logging.error("Failed to send data to LED's" + e)
            exit()

def serve(led_ip, grpc_port):
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))
    led_serv.add_LEDStateIndicatorServicer_to_server(
        LEDStateIndicator(led_ip), server)
    server.add_insecure_port('[::]:'+str(grpc_port))
    server.start()

    try:
        while True:
            time.sleep(60*60*24)
    except KeyboardInterrupt:
        server.stop(0)


if __name__ == '__main__':

    logging.basicConfig(level=logging.DEBUG)
    if 'LED_IP' in os.environ:
        LED_IP = os.environ['LED_IP']
    else:
        LED_IP = '10.4.1.85'
    if 'GRPC_PORT' in os.environ:
        GRPC_PORT = os.environ['GRPC_PORT']
    else:
        GRPC_PORT = 50051

    logging.info("Connecting LED's on ip " + LED_IP)
    logging.info("Starting gRPC on port " + str(GRPC_PORT))

    serve(LED_IP, GRPC_PORT)
