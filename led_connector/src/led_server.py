#!/usr/bin/env python


import time
import grpc
import led_connector_pb2 as led_msg
import led_connector_pb2_grpc as led_serv
from led_connector.srv import *
import rospy

# with grpc.insecure_channel('localhost:50051') as channel:
#     stub = led_serv.LEDStateIndicatorStub(channel)
#     ''' Roboter stop'''
#     #stub.setLEDState(led_msg.LEDState(color='#ff0000', frequency=0))
#     '''betriebsbereit '''
#     #stub.setLEDState(led_msg.LEDState(color='#00ff37', frequency=0))
#     '''Roboter arbeitet '''
#     #stub.setLEDState(led_msg.LEDState(color='#000dff', frequency=1))
#     '''Roboter pausiert '''
#     #stub.setLEDState(led_msg.LEDState(color='#ff8800', frequency=0))
#     # time.sleep(0.1)
#     stub.setLEDIp(led_msg.LEDConfig(ip='10.4.1.85'))
#     time.sleep(0.1)
#     #stub.resetLED(led_msg.Empty())


def led_callback(req):
    if req.state == "stop": 
        color = '#ff0000'
        frequency = 0
        data = True
    elif req.state == "betriebsbereit":
        color = "#1aff00"
        frequency = 0
        data = True
    elif req.state == "arbeitet":
        color = "#000dff"
        frequency = 1
        data = True
    elif req.state == "pausiert":
        color = "#ff8800'"
        frequency = 0
        data = True
    else:
        rospy.loginfo_once("The desired State is not defined!!!")
        color = '#ff0000'
        frequency = 0
        data = False
    print data
    print "ledResponse"
    print ledResponse
    with grpc.insecure_channel('localhost:50051') as channel:
        stub = led_serv.LEDStateIndicatorStub(channel)
        stub.setLEDState(led_msg.LEDState(color = color, frequency = frequency))
        #stub.setLEDState(led_msg.LEDState(color='#ff0000', frequency=0))
    return ledResponse(data)

def led_server():
    rospy.init_node('led_server')
    s = rospy.Service('led_server', led, led_callback)
    print "led_server is ready."
    rospy.spin()

if __name__ == "__main__":
    print type(led)
    print led
    led_server()

     



