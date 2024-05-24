#!/usr/bin/env python

import math
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import Range

import pyrealsense2 as rs

class DepthToRangeConverter:
    def __init__(self):
        rospy.init_node('depth_to_range_converter', anonymous=True)
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.range_filtered_pub = rospy.Publisher('/range_filtered_msg', Range, queue_size=10)
        self.range_pub = rospy.Publisher('/range_msg', Range, queue_size=10)
        self.error_filtered_pub = rospy.Publisher('/error_filtered_msg', Float64, queue_size=10)
        self.error_pub = rospy.Publisher('/error_msg', Float64, queue_size=10)
        self.square_size = rospy.get_param('~square_size', 1)
        self.alpha = rospy.get_param("~alpha", 0.2)
        
        self.distance_filter_out = 0
        self.last_distance_filtered = 0
        self.last_distance = 0
        
    def depth_callback(self, data):
        try:
            # Converter a imagem de profundidade para matriz numpy
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr(e)

        if self.square_size > 1:
            # Obtenha a profundidade média do quadrado de pixels no centro da imagem
            center_x = depth_image.shape[1] // 2
            center_y = depth_image.shape[0] // 2
            half_size = self.square_size // 2
            depth_square = depth_image[center_y - half_size:center_y + half_size + 1,
                           center_x - half_size:center_x + half_size + 1]
            depth_center = np.mean(depth_square)
        else:
            # Obtenha a profundidade do pixel no centro da imagem
            center_x = depth_image.shape[1] // 2
            center_y = depth_image.shape[0] // 2
            depth_center = depth_image[center_y, center_x]

        depth_in_meters = depth_center * 0.001
        
        #TO DO pegar 5 valores e fazer a média
        
        
        # filtro passa baixa para suavizar a medição
        self.distance_filter_out = (depth_in_meters * self.alpha) + (1 - self.alpha) * self.distance_filter_out
        
        
        range_filtered_msg = Range()
        range_filtered_msg.header.stamp = rospy.Time.now()
        range_filtered_msg.header.frame_id = 'depth_sensor_frame'  # Frame de referência da medição de profundidade
        range_filtered_msg.radiation_type = Range.INFRARED
        range_filtered_msg.field_of_view = 10 * math.pi / 180 # graus para radianos
        range_filtered_msg.min_range = 0.1
        range_filtered_msg.max_range = 20.0  
        range_filtered_msg.range = self.distance_filter_out
        

        self.range_filtered_pub.publish(range_filtered_msg)
        
        #publish raw data
        range_msg = Range()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.header.frame_id = 'depth_sensor_frame'
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = 10 * math.pi / 180
        range_msg.min_range = 0.1
        range_msg.max_range = 20.0
        range_msg.range = depth_in_meters
        self.range_pub.publish(range_msg)
        
        # publish filtered error
        error_filtered_msg = Float64()
        error_filtered_msg.data = abs(self.distance_filter_out - self.last_distance_filtered)
        self.last_distance_filtered = self.distance_filter_out
        self.error_filtered_pub.publish(error_filtered_msg)
        
        # publish raw error
        error_msg = Float64()
        error_msg.data = abs(depth_in_meters - self.last_distance)
        self.last_distance = depth_in_meters
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        converter = DepthToRangeConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
