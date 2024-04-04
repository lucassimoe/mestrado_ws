#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import Range

class DepthToRangeConverter:
    def __init__(self):
        rospy.init_node('depth_to_range_converter', anonymous=True)
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.range_pub = rospy.Publisher('/range_msg', Range, queue_size=10)
        self.square_size = rospy.get_param('~square_size', 2)

    def depth_callback(self, data):
        try:
            # Converter a imagem de profundidade para matriz numpy
            depth_image = self.bridge.imgmsg_to_cv2(data, 'passthrough')



        except Exception as e:
            rospy.logerr(e)
            return

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

        # Converter profundidade para metros (RealSense fornece profundidade em metros)
        depth_in_meters = depth_center * 0.001

        # Publicar a medida de profundidade no tópico /range_msg
        range_msg = Range()
        range_msg.header.stamp = rospy.Time.now()
        range_msg.header.frame_id = 'depth_sensor_frame'  # Frame de referência da medição de profundidade
        range_msg.radiation_type = Range.INFRARED
        range_msg.field_of_view = 1.047  # Ângulo de visão da câmera RealSense D435i em radianos
        range_msg.min_range = 0.1  # Distância mínima de medição da câmera RealSense D435i em metros
        range_msg.max_range = 20.0  # Distância máxima de medição da câmera RealSense D435i em metros
        range_msg.range = depth_in_meters

        self.range_pub.publish(range_msg)

if __name__ == '__main__':
    try:
        # Defina o tamanho do quadrado de pixels (square_size) para calcular a distância. Se square_size for 1, ele usará apenas um pixel.
        converter = DepthToRangeConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass