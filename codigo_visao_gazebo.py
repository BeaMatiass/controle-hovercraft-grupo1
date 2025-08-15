#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

def funcao_callback(image):
    
    try:
        # Converte a imagem ROS para formato OpenCV
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
    
    if(M['m00'] > min_pixels):
        
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(frame, (cx, cy), 30, (0, 0, 255), -1)
        cv2.line(frame, (cx, cy), (width // 2, height // 2), (0, 0, 255), 5)

        # distancia do centro do frame ao centro de distribuição de vermelho, dada em pixels
        distance = ((cx - (width // 2) )**2 + (cy - (height // 2) )**2)**(1/2)
        print(f'Distância até o centro: {distance:.2f}')

    cv2.imshow('Video', frame)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node("camera_gazebo")
    rospy.Subscriber("camera/rgb/image_raw", Image, funcao_callback)

    while not rospy.is_shutdown():
        pass

    cv2.destroyAllWindows()

