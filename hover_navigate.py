#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

def processar_pid(erro, estado, kp, ki, kd):

    # tempo -> calculo derivativo e integral
    tempo_atual = time.time()

    # primeira interação não se calcula integral e derivativo, só proporcional -> dt = 0.0
    if estado["tempo_antigo"] == 0.0:
        dt = 0.0
        estado["tempo_antigo"] = tempo_atual
    else:
        dt = tempo_atual - estado["tempo_antigo"]
        estado["tempo_antigo"] = tempo_atual
    
    # acumula o erro
    estado["integral"] += erro * dt

    # calcula ação derivativa
    if dt > 0:
        acao_derivativa = (erro - estado["erro_antigo"]) / dt
    else:
        acao_derivativa = 0.0
    
    # calcula ação de controle total
    acao_de_controle = kp * erro + ki * estado["integral"] + kd * acao_derivativa

    # define como erro_antigo o erro já computado
    estado["erro_antigo"] = erro

    return acao_de_controle



def callback_world(msg):

    global estado_angular, estado_linear, pub_speed
    global kp_angular, ki_angular, kd_angular

    lista = msg.split(",")

    dx = float(lista[0])
    tamanho_atual = float(lista[1])
    tamanho_capturado = 0.60 # proporção desejada do objeto em relação a imagem


    if tamanho_atual >= tamanho_capturado:
        acao_de_controle_linear = 0.0
        acao_de_controle_angular = 0.0
    else:
        # PID para rotação (erro: dx)
        acao_de_controle_angular = processar_pid(dx, estado_angular, kp_angular, ki_angular, kd_angular)
        # PID para linear (erro: erro_distancia)
        erro_distancia = tamanho_capturado - tamanho_atual
        acao_de_controle_linear = processar_pid(erro_distancia, estado_linear, kp_linear, ki_linear, kd_linear)

    cmd = Twist()
    cmd.linear.x = acao_de_controle_linear
    cmd.angular.z = -acao_de_controle_angular
    pub_speed.publish(cmd)

    return
 
if __name__ == '__main__':
    rospy.init_node("hover_navigate")

    # Estados do PID
    estado_angular = {"integral": 0.0, "erro_antigo": 0.0, "tempo_antigo": 0.0}
    estado_linear = {"integral": 0.0, "erro_antigo": 0.0, "tempo_antigo": 0.0}

    # Parâmetros do PID - Rotação
    kp_angular = 0.002 # ajustar
    ki_angular = 0.001 # ajustar
    kd_angular = 0.001 # ajustar

    # Parâmetros do PID - Linear
    kp_linear = 1.0 # ajustar
    ki_linear = 0.0 # ajustar
    kd_linear = 0.0 # ajustar

    pub_speed = rospy.Publisher("/speed", Twist, queue_size=1)
    rospy.Subscriber("/world", String, callback_world) # mudar nome tópico

    while not rospy.is_shutdown():
        pass
