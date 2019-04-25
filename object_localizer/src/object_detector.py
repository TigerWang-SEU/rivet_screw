#!/usr/bin/env python
from __future__ import print_function

import os
import sys

import numpy as np
import tensorflow as tf
if tf.__version__ < '1.4.0':
  raise ImportError('Please upgrade your tensorflow installation to v1.4.* or later!')

import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError

import cv2

from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
# print( sys.path )

from object_localizer_msg.msg import BBox_list
from object_localizer_msg.msg import BBox_int
from object_localizer_msg.msg import BBox_float

threshold = 0.95

######################################################################################################
# set path to detection_graph and load it
import rospkg

rospack = rospkg.RosPack()
PATH_TO_CKPT = rospack.get_path('object_localizer') + '/model/model_2/frozen_inference_graph.pb'
detection_graph = tf.Graph()
with detection_graph.as_default():
    od_graph_def = tf.GraphDef()
    with tf.gfile.GFile( PATH_TO_CKPT, 'rb' ) as fid:
        serialized_graph = fid.read()
        od_graph_def.ParseFromString( serialized_graph )
        tf.import_graph_def( od_graph_def, name='' )
print( '***load saved graph from: ', PATH_TO_CKPT )

# list of the strings that is used to add correct label for each box.
PATH_TO_LABELS = rospack.get_path('object_localizer') + '/model/model_2/object_map.pbtxt'
NUM_CLASSES = 2
label_map = label_map_util.load_labelmap( PATH_TO_LABELS )
categories = label_map_util.convert_label_map_to_categories( label_map, max_num_classes = NUM_CLASSES, use_display_name = True )
category_index = label_map_util.create_category_index( categories )
print( '***load category_index :', category_index )
#######################################################################################################
# create a tensorflow session for detection_graph
sess = tf.Session( graph = detection_graph )

# set the input (image_tensor) and output (tensor_dict) map for detection_graph
tensor_dict = {}
with detection_graph.as_default():
    image_tensor = tf.get_default_graph().get_tensor_by_name( 'image_tensor:0' )
    ops = tf.get_default_graph().get_operations()
    all_tensor_names = { output.name for op in ops for output in op.outputs }
    for key in [ 'num_detections', 'detection_boxes', 'detection_scores', 'detection_classes', 'detection_masks' ]:
        tensor_name = key + ':0'
        if tensor_name in all_tensor_names:
            tensor_dict[ key ] = tf.get_default_graph().get_tensor_by_name( tensor_name )
            print( '***Add [', tensor_name, '] to tensor_dict' )
# print ( type( image_tensor ), type( tensor_dict ) )
#######################################################################################################
class image_converter:

    def __init__( self ):
        self.image_pub = rospy.Publisher( "/object_localizer/localize_image", Image, queue_size = 100 )
        self.bbox_pub = rospy.Publisher( "/object_localizer/bbox_list", BBox_list, queue_size = 100 )
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber( "/camera/color/image_raw", Image, self.image_callback )
        # self.cloud_sub = rospy.Subscriber( "/camera/depth_registered/points", PointCloud2, self.cloud_callback )

    # def cloud_callback( self, data ):
    #     frame_id = data.header.frame_id
    #     # print( 'Frame id for cloud msg is ', frame_id )
    #     height = data.height
    #     width = data.width
    #     print( 'The point cloud has height =', height, 'width =', width )

    def image_callback( self, data ):
        # print( 'Receive image at time:', data.header.stamp )
        try:
            image_np = self.bridge.imgmsg_to_cv2( data, "rgb8" )
        except CvBridgeError as e:
            print( e )
        ( im_width, im_height, depth ) = image_np.shape
        # print( 'Input image has shape:', im_width, im_height, depth )

        ###########################################################################################
        # cv2.imshow( "Image window", image_np )
        # cv2.waitKey( 3 )

        image_np_expanded = np.expand_dims( image_np, axis=0 )
        # Actual detection.
        output_dict = sess.run( tensor_dict, feed_dict = { image_tensor: image_np_expanded } )

        # all outputs are float32 numpy arrays, so convert types as appropriate
        output_dict[ 'num_detections' ] = int( output_dict[ 'num_detections' ][ 0 ] )
        output_dict[ 'detection_classes' ] = output_dict[ 'detection_classes' ][ 0 ].astype( np.uint8 )
        output_dict[ 'detection_boxes' ] = output_dict[ 'detection_boxes' ][ 0 ]
        output_dict[ 'detection_scores' ] = output_dict[ 'detection_scores' ][ 0 ]
        if 'detection_masks' in output_dict:
            output_dict[ 'detection_masks' ] = output_dict[ 'detection_masks' ][ 0 ]

        output_dict[ 'detection_scores' ][ output_dict[ 'detection_scores' ] < threshold ] = 0.0
        # print( output_dict[ 'detection_scores' ][ output_dict[ 'detection_scores' ] > threshold ] )
        # if output_dict[ 'detection_scores' ][ 0 ] < 0.5 and output_dict[ 'detection_scores' ][ 0 ] > 0.1:
        #     output_dict[ 'detection_scores' ][0 ] = 0.51
        # if output_dict[ 'detection_scores' ][ 1 ] < 0.5 and output_dict[ 'detection_scores' ][ 1 ] > 0.1:
        #     output_dict[ 'detection_scores' ][ 1 ] = 0.51
        # if output_dict[ 'detection_scores' ][ 2 ] < 0.5 and output_dict[ 'detection_scores' ][ 2 ] > 0.1:
        #     output_dict[ 'detection_scores' ][ 2 ] = 0.51

        # Visualization of the results of a detection.
        vis_util.visualize_boxes_and_labels_on_image_array( image_np,
                                                            output_dict[ 'detection_boxes' ],
                                                            output_dict[ 'detection_classes' ],
                                                            output_dict[ 'detection_scores' ],
                                                            category_index,
                                                            instance_masks = output_dict.get( 'detection_masks' ),
                                                            use_normalized_coordinates = True,
                                                            line_thickness = 6 )

        # Publish the bounding box list messages
        # Filter out bounding boxs have confidence score less than 0.99
        result_box = output_dict[ 'detection_boxes' ][ output_dict[ 'detection_scores' ] > threshold, : ]
        detection_class_list = output_dict[ 'detection_classes' ][ output_dict[ 'detection_scores' ] > threshold ]
        xmin = result_box[:, 0]
        xmax = result_box[:, 2]
        ymin = result_box[:, 1]
        ymax = result_box[:, 3]
        (x1, x2, y1, y2) = (xmin * im_width, xmax * im_width, ymin * im_height, ymax * im_height)
        # print(x1, x2, y1, y2)

        bbox_list = BBox_list()
        # print(type(bbox_list.BBox_list))
        bbox_list.header.frame_id = data.header.frame_id
        bbox_list.header.stamp = data.header.stamp
        for idx, value in enumerate(x1):
            new_bbox = BBox_int()
            new_bbox.x1 = int( value )
            new_bbox.x2 = int( x2[idx] )
            new_bbox.y1 = int( y1[idx] )
            new_bbox.y2 = int( y2[idx] )
            bbox_list.BBox_list_int.append( new_bbox )
            bbox_list.class_list.append( detection_class_list[idx] )
        self.bbox_pub.publish( bbox_list )
        ###########################################################################################

        try:
            self.image_pub.publish( self.bridge.cv2_to_imgmsg( image_np, "rgb8" ) )
        except CvBridgeError as e:
            print( e )

def main(args):
    ic = image_converter()
    rospy.init_node( 'object_localizer', anonymous=True )
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print( "Shutting down" )
    cv2.destroyAllWindows()
    sess.close()

if __name__ == '__main__':
    main(sys.argv)
