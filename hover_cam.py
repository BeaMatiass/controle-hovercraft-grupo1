#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

bridge = CvBridge()
contador = 0

def funcao_callback(image):

    global contador
    contador += 1

    if(contador % 6 != 0):
        return
    
    try:
        frame = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
    except Exception as e:
        rospy.logerr(f"Erro ao converter imagem: {e}")
        return

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, (0, 150, 50), (10, 255, 255))
    mask2 = cv2.inRange(hsv, (160, 150, 50), (179, 255, 255))
    mask = cv2.bitwise_or(mask1, mask2)
    M = cv2.moments(mask)
    
    height, width, _ = frame.shape 
    min_pixels = height*width*0.135
    num_pixels = cv2.countNonZero(mask)
    

    if(num_pixels > min_pixels): 
        
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        dx = cx - (width // 2)
        tamanho_atual = num_pixels / (height * width)

        msg = f"{dx},{tamanho_atual}"
        pub_world.publish(msg) # publisher em /world -> alterar depois para publicar em subtópicos as três informações separadamente

        cv2.circle(frame, (cx, cy), 30, (0, 0, 255), -1)
        cv2.line(frame, (cx, cy), (width // 2, height // 2), (0, 0, 255), 5)

    cv2.imshow('Video', frame)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node("camera_gazebo")

    pub_world = rospy.Publisher("/world", String, queue_size=1)
    rospy.Subscriber("camera/rgb/image_raw", Image, funcao_callback, queue_size=1)

    while not rospy.is_shutdown():
        pass

    cv2.destroyAllWindows()

