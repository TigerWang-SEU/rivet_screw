#!/usr/bin/env python3

import sys
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import rospy
import time
import os
import sys
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
import Open_Protocol.open_protocol as OpenProtocol
import Open_Protocol.networking as networking

if __name__ == "__main__":
    print ('new_nut')
    new_nut = rospy.ServiceProxy('new_nut', Empty)
    new_nut()
    rospy.sleep(1)

    print ('start_scrweing')
    start_screwing = rospy.ServiceProxy('start_screwing', Empty)
    start_screwing()
    rospy.sleep(10)

    print("stop the screwing")
    stop = rospy.ServiceProxy('stop', Empty)
    stop()
