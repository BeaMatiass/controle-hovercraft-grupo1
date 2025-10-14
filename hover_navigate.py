#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from codigo_pid import controle_pid

# Objetos globais
pub_speed = None
pub_angular = None
pub_linear = None

# Variáveis globais
dx = None
tamanho_atual = None
tamanho_capturado = 0.60

ultimo_tempo_visto = None

def callback_dx(msg):
    global dx, ultimo_tempo_visto
    dx = msg.data
    ultimo_tempo_visto = rospy.Time.now()
    processar_controle()
    return

def callback_tamanho(msg):
    global tamanho_atual, ultimo_tempo_visto
    tamanho_atual = msg.data
    ultimo_tempo_visto = rospy.Time.now()
    processar_controle()
    return


def processar_controle():
    global dx, tamanho_atual
    global pid_angular, pid_linear
    global pub_linear, pub_angular
    global tamanho_capturado

    if dx is None or tamanho_atual is None:
        return

    if tamanho_atual >= tamanho_capturado:
        acao_linear = 0.0
        acao_angular = 0.0
    else:
        acao_angular = pid_angular.processar_pid(dx)
        erro_distancia = tamanho_capturado - tamanho_atual
        acao_linear = pid_linear.processar_pid(erro_distancia)

    pub_linear.publish(acao_linear)
    pub_angular.publish(-acao_angular)
    return

def controle_periodico():
    global ultimo_tempo_visto

    if ultimo_tempo_visto is None:
        pub_linear.publish(0.0)
        pub_linear.publish(0.3)
        return
    
    tempo_sem_objeto = (rospy.Time.now() - ultimo_tempo_visto)
    if tempo_sem_objeto > 1.0:
        pub_linear.publish(0.0)
        pub_linear.publish(0.3)
        return

 
if __name__ == '__main__':
    rospy.init_node("hover_navigate")

    # Inicializa o PID
    pid_angular = controle_pid(kp = 0.002, ki = 0.001, kd = 0.001)
    pid_linear = controle_pid(kp = 1.0, ki = 0.0, kd = 0.0)

    # Publishers
    pub_linear = rospy.Publisher("/speed/velocidade_linear", Float32, queue_size=1)
    pub_angular = rospy.Publisher("/speed/velocidade_angular", Float32, queue_size=1)

    # Subscribers
    rospy.Subscriber("/dados_da_camera/distancia_x", Float32, callback_dx)
    rospy.Subscriber("/dados_da_camera/tamanho_atual", Float32, callback_tamanho)

    # Timer periódico (busca quando não há dados obtidos no subscriber -> sem objeto na tela)
    rospy.Timer(rospy.Duration(0.1), controle_periodico)

    rospy.spin()
