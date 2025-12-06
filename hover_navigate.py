#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
from pid import controle_pid

# Objetos globais
pub_angular = None
pub_linear = None

# Variáveis globais
dx = None
tamanho_atual = None
vel_angular_real = 0.0
tamanho_capturado = 0.60 # ajustar depois

ultimo_tempo_visto = 0.0

def callback_dx(msg):
    global dx, ultimo_tempo_visto
    dx = msg.data
    ultimo_tempo_visto = rospy.Time.now()
    return

def callback_tamanho(msg):
    global tamanho_atual, ultimo_tempo_visto
    tamanho_atual = msg.data
    return

# Precisamos, nessa parte, dos valores dados pelos sensores da PIX. Não tem como aplicar o PID 
# para tentar manter o hovercraft na velocidade linear a angular correta se não temos como saber
# a velocidade linear e angular do hover.

def callback_mavros_vel(msg):
    global vel_angular_real
    vel_angular_real = msg.twist.angular.z
    return


def controle_periodico(_event):
    global ultimo_tempo_visto

# Perdido:

    # tempo desde a última detecção da câmera
    tempo_sem_objeto = (rospy.Time.now() - ultimo_tempo_visto).to_sec()

    # Se nunca viu o objeto / faz tempo que não o viu, entra em "procura" (gira!)
    if ultimo_tempo_visto == 0.0 or tempo_sem_objeto > 3.0:
        pub_linear.publish(0.3)
        pub_angular.publish(0.3) # valores arbitrários (depois corrigir)
        return

# Encontrado:

    # PID da distância linear (é preciso mudar)
    velocidade_linear = pid_distance.processar_pid(tamanho_atual)
    pub_linear.publish(velocidade_linear)

    # PID de dx com feedback da PIX
    vel_angular_desejada = pid_dx.processar_pid(dx)
    erro_vel = vel_angular_desejada - vel_angular_real
    comando_angular = pid_vel.processar_pid(erro_vel)
    pub_angular.publish(comando_angular)


if __name__ == '__main__':
    rospy.init_node("hover_navigate")

    # PID para distância (velocidade linear)
    pid_distance = controle_pid(
        kp = 0.1, 
        ki = 0.1, 
        kd = 0.1,
        setpoint = tamanho_capturado,
        lower_limit=0.0,
        upper_limit=1.0
    )
    
    # PID externo (dx -> velocidade angular desejada)
    pid_dx = controle_pid(
        kp = 0.1,
        ki = 0.1,
        kd = 0.1,
        setpoint = 0.0,
        lower_limit = -1.0,
        upper_limit = 1.0
    )

    # PID interno (velocidade angular desejada x velocidade real)
    pid_vel = controle_pid(
        kp = 0.4,
        ki = 0.1,
        kd = 0.05,
        setpoint = 0.0, # visa zerar a diferença das velocidades
        lower_limit = -1.0,
        upper_limit = 1.0
    )

    # Publishers
    pub_linear = rospy.Publisher("/speed/velocidade_linear", Float32, queue_size=1)
    pub_angular = rospy.Publisher("/speed/velocidade_angular", Float32, queue_size=1)

    # pub_pid_distance = rospy.Publisher("/pid/distance", Float32, queue_size=1)
    # pub_pid_dx = rospy.Publisher("/pid/dx", Float32, queue_size=1)
    # pub_pid_angular = rospy.Publisher("/pid/angular", Float32, queue_size=1)

    # Subscribers
    rospy.Subscriber("/dados_da_camera/distancia_x", Float32, callback_dx)
    rospy.Subscriber("/dados_da_camera/tamanho_atual", Float32, callback_tamanho)
    rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, callback_mavros_vel)

    # Timer periódico (busca quando não há dados obtidos no subscriber -> sem objeto na tela)
    rospy.Timer(rospy.Duration(0.1), controle_periodico)

    rospy.spin()
    
