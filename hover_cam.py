#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

bridge = CvBridge()
contador = 0

def funcao_callback(image):
    # Buffer
    global contador
    contador += 1

    if(contador % 6 != 0):
        return
    
    # Conversão
    try:
        frame = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
    except Exception as e:
        rospy.logerr(f"Erro ao converter imagem: {e}")
        return

    # Algoritmo reconhecimento pixels vermelhos
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, (0, 150, 50), (10, 255, 255))
    mask2 = cv2.inRange(hsv, (160, 150, 50), (179, 255, 255))
    mask = cv2.bitwise_or(mask1, mask2)
    M = cv2.moments(mask)
    
    height, width, _ = frame.shape 
    min_pixels = height*width*0.135
    num_pixels = cv2.countNonZero(mask)
    
    # Se houver objeto vermelho na tela, publica suas informações
    if(num_pixels > min_pixels and (M['m00'] != 0)): 

        cx = int(M['m10']/M['m00'])
        dx = cx - (width // 2)

        # normaliza dx para -1...1 (centro = 0, bordas ~ ±1)
        dx_normalizado = float(dx) / (width / 2.0)

        # coloquei essa sugestão do chat
        if dx_normalizado > 1.0:
            dx_normalizado = 1.0
        if dx_normalizado < -1.0:
            dx_normalizado = -1.0

        tamanho_atual = num_pixels / (height * width)

        pub_distancia_x.publish(dx_normalizado)
        pub_tamanho_atual.publish(tamanho_atual)

if __name__ == '__main__':
    rospy.init_node("hover_cam")

    # Publishers
    pub_distancia_x = rospy.Publisher("/dados_da_camera/distancia_x", Float32, queue_size=1)
    pub_tamanho_atual = rospy.Publisher("/dados_da_camera/tamanho_atual", Float32, queue_size=1)

    # Subscribers
    # O que está abaixo eu encontrei do pacote usb_cam, um driver ROS que transforma frames reais em um tópico image_raw
    rospy.Subscriber("/usb_cam/rgb/image_raw", Image, funcao_callback, queue_size=1)

    rospy.spin()
