#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node("camera_webcam_pub")

    pub = rospy.Publisher("/camera/rgb/image_raw", Image, queue_size=1)
    bridge = CvBridge()

    cap = cv2.VideoCapture(2)  # Tenta abrir a webcam

    if not cap.isOpened():
        rospy.logerr("Não foi possível abrir a webcam!")
        return

    rate = rospy.Rate(30)  # 30 FPS

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Falha ao capturar frame!")
            continue

        # --- Mostra a imagem na tela ---
        cv2.imshow("Webcam", frame)
        cv2.waitKey(1)  # Necessário para atualizar a janela (1 ms)

        # --- Publica no ROS ---
        img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(img_msg)

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
