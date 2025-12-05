#!/usr/bin/env python3

import rospy
import cv2
import numpy as np  # Necessário para matrizes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# --- DADOS DE CALIBRAÇÃO ---
# Matriz Intrínseca (Camera Matrix - K)
K = np.array([
    [575.531975, 0.000000, 262.859918],
    [0.000000, 574.741823, 258.231277],
    [0.000000, 0.000000, 1.000000]
])

# Coeficientes de Distorção (D)
D = np.array([0.080291, -0.138517, -0.002353, -0.002475, 0.000000])

# Matriz de Projeção (Projection Matrix - P)
# Usamos apenas a parte 3x3 (rotacional/escala) para o newCameraMatrix
P = np.array([
    [581.062867, 0.000000, 260.924261],
    [0.000000, 582.487228, 257.382226],
    [0.000000, 0.000000, 1.000000]
])

def main():
    rospy.init_node("camera_webcam_pub")

    pub = rospy.Publisher("/camera/rgb/image_rect_color", Image, queue_size=1) # Mudei o tópico para indicar que é retificada
    bridge = CvBridge()

    cap = cv2.VideoCapture(3) 

    if not cap.isOpened():
        rospy.logerr("Não foi possível abrir a webcam!")
        return

    rate = rospy.Rate(30) 

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Falha ao capturar frame!")
            continue

        # --- APLICA A CALIBRAÇÃO (UNDISTORT) ---
        # frame: imagem original
        # K: matriz original da câmera
        # D: distorção
        # None: retificação (não usada aqui pois é monocular simples)
        # P: nova matriz de câmera (como a imagem deve ficar após o corte/zoom ideal)
        frame_rectified = cv2.undistort(frame, K, D, None, P)
        
        # frame_rectified = cv2.undistort(frame, K, D)
        # --- Mostra a imagem na tela (Comparação) ---
        # Mostra a imagem corrigida
        cv2.imshow("Webcam Retificada", frame_rectified)
        
        cv2.waitKey(1) 

        # --- Publica no ROS a imagem JÁ CORRIGIDA ---
        img_msg = bridge.cv2_to_imgmsg(frame_rectified, encoding="bgr8")
        pub.publish(img_msg)

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()