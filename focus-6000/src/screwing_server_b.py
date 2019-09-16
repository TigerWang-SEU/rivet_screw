#!/usr/bin/env python
# coding: utf8
# find the path of the parent directory of the acutal python file
import logging

import rospy
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import os
import sys
import inspect

import grpc
import time
## import go generated files
import festo_pb2
import festo_pb2_grpc

currentdir = os.path.dirname ( os.path.abspath ( inspect.getfile ( inspect.currentframe () ) ) )
parentdir = os.path.dirname ( currentdir )
sys.path.insert ( 0, parentdir )

import Open_Protocol.open_protocol as OpenProtocol
import Open_Protocol.networking as networking

def pset_param ( pset_number, loosening_speed, loosening_torque, loosening_angle, tightening_speed, tightening_torque ):
    rospy.set_param ( 'pset_id',"3" )
    if pset_number!=1:
      rospy.set_param ( 'pset_number',pset_number )
      if loosening_speed>=50 and loosening_speed<=750:
          rospy.set_param ( 'loosening_speed',loosening_speed )
      else:
          print 'loosening_speed: the input is outside of the limit'
      if loosening_torque>=0 and loosening_torque<=300:
          rospy.set_param ('loosening_torque',loosening_torque)
      else:
          print 'loosening_torque: the input is outside of the limit'
      if loosening_angle>=0 and loosening_angle<=10800:
          rospy.set_param ('loosening_angle',loosening_angle)
      else:
          print 'loosening_angle: the input is outside of the limit'
      if tightening_speed>=50 and tightening_speed<=100:
          rospy.set_param ('tightening_speed',tightening_speed)
      else:
          print 'tightening_speed: the input is outside of the limit'
      if tightening_torque>=62.5 and tightening_torque<=275:
          rospy.set_param ('tightening_torque',tightening_torque)
      else:
          print 'tightening_torque: the input is out of the limit'
    else:
      print 'unvalid pset_number'

def pset_setting ():
    pset_number = rospy.get_param ('pset_number')
    loosening_speed = rospy.get_param ('loosening_speed')
    loosening_torque = rospy.get_param ('loosening_torque')
    loosening_angle = rospy.get_param ('loosening_angle')
    tightening_speed = rospy.get_param ('tightening_speed')
    tightening_torque = rospy.get_param ('tightening_torque')
    op.pset_message ( pset_number, loosening_speed, loosening_torque, loosening_angle, tightening_speed, tightening_torque ) #create/overwrite a pset and generate the message to be send for tightening/loosening process
    tightening_torque = sock.sendMID(op.message('Tightening Program Message download','001',op.tightening_torque_data,' ','  ')) #the target torque of the tightening process /cNm
    tightening_speed = sock.sendMID(op.message('Tightening Program Message download','001',op.tightening_speed_data,' ','  ')) #the rotation velocity of the tightening process /rpm
    loosening_torque = sock.sendMID(op.message('Tightening Program Message download','001',op.loosening_torque_data,' ','  ')) #the loosening torque /cNm
    loosening_speed = sock.sendMID(op.message('Tightening Program Message download','001',op.loosening_speed_data,' ','  '))  #/rpm
    loosening_angle = sock.sendMID(op.message('Tightening Program Message download','001',op.loosening_angle_data,' ','  ')) #/ °

def select_pset():
    pset_id = rospy.get_param('pset_id').zfill(3)
    print ('pset_id',pset_id)
    select_pset=sock.sendMID(op.message('Select_Parameter_set', '001', pset_id,' ','  '))
    return select_pset

def wake_up():
    wake_up = sock.sendMID(op.message('Digital Input Function', '001', '44',' ','  '))

def stand_by():
    stand_by = sock.sendMID(op.message('Digital Input Function', '001', '38',' ','  '))

# define the service recall functions for lift table
def start_lift_table ( req ):
    with grpc.insecure_channel("localhost:50051") as channel:
        client = festo_pb2_grpc.valveControllerStub ( channel )
        client.openValve ( festo_pb2.ValveRequest ( valveNo = 3 ) )
    return {}

def stop_lift_table ( req ):
    with grpc.insecure_channel("localhost:50051") as channel:
        client = festo_pb2_grpc.valveControllerStub ( channel )
        client.closeValve ( festo_pb2.ValveRequest ( valveNo = 3 ) )
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

# the service recall function for screwing
def start_screwing ( req ):
    start_tightening = sock.sendMID(op.message('Digital Input Function', '001', '3', ' ', '  '))
    return {}

def stop_screwing ( req ):
    stop = sock.sendMID(op.message('Digital Input Function', '001', '5', ' ', '  '))
    return {}

# the service recall function for loosening
def start_loosening ( req ):
    start_loosening = sock.sendMID(op.message('Digital Input Function', '001','3', ' ', '  '))
    return {}

if __name__ == "__main__":

    # set necessary parameters
    pset_param ( 77, 750, 223.4, 2223, 90, 123.4 )
    rospy.init_node ( 'rivet_tool_server' )

    sock = networking.opProtocol ()
    sock.IP = "192.168.2.33"
    # sock.IP = "10.4.1.64"
    sock.PORT = 4545
    sock.BUFFER = 1024
    sock.connect ( sock.IP, sock.PORT )
    op = OpenProtocol.OpCon ()
    pset_setting ()
    wake_up ()

    start_lift_table = rospy.Service ( 'start_lift_table', Empty, start_lift_table )
    stop_lift_table = rospy.Service ( 'stop_lift_table', Empty, stop_lift_table )
    new_nut = rospy.Service ( 'new_nut', Empty, new_nut )
    stop_new_nut = rospy.Service ( 'stop_new_nut', Empty, stop_new_nut )
    start_screwing = rospy.Service ( 'start_screwing', Empty, start_screwing )
    stop_screwing = rospy.Service ( 'stop_screwing', Empty, stop_screwing )

    rospy.spin ()
    stand_by ()
    print ( "stand by" )
    sock.close()
    print ( "disconnect" )
