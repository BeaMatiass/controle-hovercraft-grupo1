#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
# from math import atan
from math import asin
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
    lower_red1 = np.array([0,   90,  90])
    upper_red1 = np.array([10,  255, 255])

    lower_red2 = np.array([170, 90,  90])
    upper_red2 = np.array([179, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    M = cv2.moments(mask)
    
    height, width, _ = frame.shape 
    min_pixels = height*width*0.001
     # ajustar tanto mínimo de pixels
    num_pixels = cv2.countNonZero(mask)
    
    # Se houver objeto vermelho na tela, publica suas informações
    if(num_pixels > min_pixels and (M['m00'] != 0)): 

        # calculos relativos à dx
        cx = int(M['m10']/M['m00'])
        dx = cx - (width // 2)
        dx_normalizado = float(dx) / (width / 2.0)

        if dx_normalizado > 1.0:
            dx_normalizado = 1.0
        if dx_normalizado < -1.0:
            dx_normalizado = -1.0
        
        tamanho_atual = num_pixels / (height * width) # pixels

        # lógica de contorno
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(contornos) == 0:
            return

        contorno_maior = max(contornos, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(contorno_maior)
        

        # determinar distância z e ângulo
        altura_pixels_objeto = float(h)
        altura_real_objeto = 19.5 # cm
        distancia_focal = 582.7 # pixels

        distancia_z = altura_real_objeto * (distancia_focal / altura_pixels_objeto) # advém da semelhaça de triângulos
        
        arg_asin = dx / distancia_focal 
        arg_asin = max(min(arg_asin, 1.0), -1.0)
        angulo_atual = asin(arg_asin)
        # angulo_atual = atan(dx / distancia_focal)

        # --- VISUALIZAÇÃO ---
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        # LINHA ADICIONADA: Imprime a distância na tela
        cv2.putText(frame, f"{distancia_z:.1f} cm", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        # --------------------

        pub_distancia_x.publish(dx_normalizado)
        pub_tamanho_atual.publish(tamanho_atual)
        pub_angulo.publish(angulo_atual)
        pub_distancia_z.publish(distancia_z)

    # VISUALIZAÇÃO (Fora do if para ver a câmera sempre)
    cv2.imshow("Mascara", mask)
    cv2.imshow("Visao Final", frame)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node("hover_cam")

    # Publishers
    pub_distancia_x = rospy.Publisher("/dados_da_camera/distancia_x", Float32, queue_size=1)
    pub_tamanho_atual = rospy.Publisher("/dados_da_camera/tamanho_atual", Float32, queue_size=1)
    pub_angulo = rospy.Publisher("/dados_da_camera/angulo_atual", Float32, queue_size=1)
    pub_distancia_z = rospy.Publisher("/dados_da_camera/distancia_z", Float32, queue_size=1)

    # Subscribers
    rospy.Subscriber("/camera/rgb/image_raw", Image, funcao_callback, queue_size=1)

    rospy.spin()
