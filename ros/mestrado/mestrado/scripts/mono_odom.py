import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Pose, Twist, PoseStamped
from nav_msgs.msg import Odometry

# import quaternion
from tf.transformations import quaternion_from_matrix

from sensor_msgs.msg import CameraInfo

# Variáveis globais float
Rpose = np.eye(3)
# follow the same format as the translation
tpose = np.array([[0], [0], [0]])

def calculate_odometry(prev_image, curr_image):
    global Rpose, tpose
    # Extrair pontos de interesse (features) e descritores das imagens
    orb = cv2.ORB_create()
    kp1, des1 = orb.detectAndCompute(prev_image, None)
    kp2, des2 = orb.detectAndCompute(curr_image, None)
    
    # Se não houver pontos de interesse suficientes, retorne a odometria anterior
    if len(kp1) < 9 or len(kp2) < 9:
        rospy.loginfo("kp1: %s", len(kp1))
        return Rpose, tpose
    # Realizar correspondência entre os pontos de interesse
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    matches = bf.knnMatch(des1, des2, k=2)
    
    # Filtrar os matches para obter os pontos correspondentes
    matches = [m for m, n in matches if m.distance < 0.75 * n.distance]
    
    # converter de keypoints para points
    prev_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    curr_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
    
    # Se não houver pontos correspondentes suficientes, retorne a odometria anterior
    if len(curr_pts) < 9:
        rospy.loginfo("curr_pts: %s", len(curr_pts))
        return Rpose, tpose
    # converter de pontos para homogêneos
    prev_pts_hom = np.array(prev_pts).T.reshape(2, -1)
    curr_pts_hom = np.array(curr_pts).T.reshape(2, -1)
    
    # calcular matriz essensial
    E, mask = cv2.findEssentialMat(curr_pts, prev_pts, matrix_K, cv2.RANSAC, 0.999, 0.1)
    
    _, R, t, mask = cv2.recoverPose(E, curr_pts, prev_pts, matrix_K, mask=mask)
    
    if R.trace() > 0:
        # rospy.loginfo("Rpose: %s", Rpose)
        # rospy.loginfo("tpose: %s", tpose)
        # rospy.loginfo("R: %s", R)
        # rospy.loginfo("t: %s", t)
        # rospy.loginfo("rpose.dot(t): %s", Rpose.dot(t))
        Rpose = R.dot(Rpose)
        tpose = tpose + Rpose.dot(t)
        
        
    return Rpose, tpose


def image_callback(msg):
    global prev_image, pub_odom
    bridge = CvBridge()
    curr_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    curr_image = cv2.cvtColor(curr_image, cv2.COLOR_BGR2RGB)

    # Se houver uma imagem anterior, calcule a odometria
    if prev_image is not None:
        
        R, t = calculate_odometry(prev_image, curr_image)
        # rospy.loginfo("t: %s", t)
        # to homogeneous matrix with translation 0
        Rh = np.eye(4)
        Rh[:3, :3] = R
        
        # transform to quaternion
        q = quaternion_from_matrix(Rh)
        
        # Publicar os dados de odometria em um tópico ROS
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'camera_link'
        odom_msg.pose.pose.position.x = t[0][0]
        odom_msg.pose.pose.position.y = t[1][0]
        odom_msg.pose.pose.position.z = t[2][0]
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        pub_odom.publish(odom_msg)

    # Atualizar a imagem anterior
    prev_image = curr_image
    
#define matrix_K
def camera_info_callback(msg):
    global matrix_K
    matrix_K = np.array(msg.K).reshape(3, 3)

def main():
    global prev_image, pub_odom
    rospy.init_node('monocular_odometry_node', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.Subscriber('/camera/color/camera_info', CameraInfo, camera_info_callback)
    pub_odom = rospy.Publisher('/camera/odom_mono', Odometry, queue_size=10)
    prev_image = None

    rospy.spin()

if __name__ == '__main__':
    main()