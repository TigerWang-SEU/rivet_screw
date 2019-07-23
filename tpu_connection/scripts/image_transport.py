#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import the grpc files
import grpc
import image_msg_pb2_grpc
import image_msg_pb2
#import bbox message definitions
from object_localizer_msg.msg import *

class ImageTransport:
  def __init__(self):
    # create ros publisher, subscriber and cv_bridge
    self.node_rate = 1
    # topic name should be changed to realsense camera topic when running on the platform
    self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
    #self.image_sub = rospy.Subscriber('/cv_camera/image_raw', Image, self.callback)
    self.image_pub = rospy.Publisher('/object_localizer/localize_image', Image, queue_size = 100)
    self.bbox_pub =  rospy.Publisher('/object_localizer/bbox_list', BBox_list, queue_size = 100)
    self.bridge = CvBridge()

    # create grpc channel and stub
    self.object_channel = grpc.insecure_channel("localhost:50052")
    self.stub =  image_msg_pb2_grpc.DetectorStub(self.object_channel)

  def callback(self,data):
    try:
      # subscribe to ros_image and convert it to opencv_image
      cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
      ( im_width, im_height, depth ) = cv_image.shape
    except CvBridgeError as e:
      print(e)

    # convert opencv_image to grpc_msg
    _, img_jpg = cv2.imencode('.jpg', cv_image)
    grpc_msg = image_msg_pb2.Image(
      data = img_jpg.tostring()
    )
    # send a request the server for detection results
    detection = self.stub.detectImage(grpc_msg)

    #create a bbox with the response from TPU
    bbox_list = BBox_list()
    bbox_list.header.frame_id = data.header.frame_id
    bbox_list.header.stamp = data.header.stamp


    for box in detection.objects:
      border_left = 420
      if(box.xmin>border_left and im_height-box.xmax>10):
          cv2.line(cv_image,(border_left,0),(border_left,im_height),(255,0,0),5)
          cv2.rectangle(cv_image, (box.xmin, box.ymin), (box.xmax, box.ymax), (0, 0, 0), 1)
          cv2.putText(cv_image, box.label, (box.xmin + 5 , box.ymin - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1, cv2.LINE_AA)
          new_bbox = BBox_int()
          new_bbox.x1 = int( box.ymin )
          new_bbox.x2 = int( box.ymax )
          new_bbox.y1 = int( box.xmin )
          new_bbox.y2 = int( box.xmax )
          bbox_list.BBox_list_int.append( new_bbox )
          try:
              self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
              print(detection)
              self.bbox_pub.publish( bbox_list )
          except CvBridgeError as e:
              print(e)
      else:
          print("ignore bounding_box")

def main(args):
  rospy.init_node('tpu_connection', anonymous=True)
  ImageTransport()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
