#!/usr/bin/env python
# coding: utf8
# find the path of the parent directory of the acutal python file
import logging

import rospy
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

import grpc
import time
import festo_pb2
import festo_pb2_grpc

# define the service recall functions for lift table
def start_lift_table ( req ):
    with grpc.insecure_channel("localhost:50051") as channel:
        client = festo_pb2_grpc.valveControllerStub ( channel )
        client.openValve ( festo_pb2.ValveRequest ( valveNo = 2 ) )
    return {}

def stop_lift_table ( req ):
    with grpc.insecure_channel("localhost:50051") as channel:
        client = festo_pb2_grpc.valveControllerStub ( channel )
        client.closeValve ( festo_pb2.ValveRequest ( valveNo = 2 ) )
    return {}

# define the service recall functions for pump new rivet
def new_nut ( req ):
    with grpc.insecure_channel("localhost:50051") as channel:
        client = festo_pb2_grpc.valveControllerStub ( channel )
        client.openValve ( festo_pb2.ValveRequest ( valveNo = 1 ) )
    return {}

def stop_new_nut ( req ):
    with grpc.insecure_channel("localhost:50051") as channel:
        client = festo_pb2_grpc.valveControllerStub ( channel )
        client.closeValve ( festo_pb2.ValveRequest ( valveNo = 1 ) )
    return {}

if __name__ == "__main__":

    rospy.init_node ( 'festo_valve_server' )
    start_lift_table = rospy.Service ( 'start_lift_table', Empty, start_lift_table )
    stop_lift_table = rospy.Service ( 'stop_lift_table', Empty, stop_lift_table )
    new_nut = rospy.Service ( 'new_nut', Empty, new_nut )
    stop_new_nut = rospy.Service ( 'stop_new_nut', Empty, stop_new_nut )

    rospy.spin ()
