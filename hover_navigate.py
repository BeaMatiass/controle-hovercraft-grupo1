#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from pid import PIDController

# Objetos globais
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
    return

def callback_tamanho(msg):
    global tamanho_atual, ultimo_tempo_visto
    tamanho_atual = msg.data
    ultimo_tempo_visto = rospy.Time.now()
    return

# Precisamos, nessa parte, dos valores dados pelos sensores da PIX. Não tem como aplicar o PID 
# para tentar manter o hovercraft na velocidade linear a angular correta se não temos como saber
# a velocidade linear e angular do hover.

def controle_periodico(_event):
    global ultimo_tempo_visto

    if ultimo_tempo_visto is None:
        pub_linear.publish(0.3)
        pub_angular.publish(0.3)
        return
    
    tempo_sem_objeto = (rospy.Time.now() - ultimo_tempo_visto).to_sec()

    if tempo_sem_objeto > 1.0:
        pub_linear.publish(0.0)
        pub_angular.publish(0.3)
        return

    if tamanho_atual is None or dx is None:
        return

    if tempo_sem_objeto < 3.0:  # Mesmo que o objeto suma da tela, o hovercraft continue se movendo na mesma direção por um tempo
        # Controle PID para distância
        pub_pid_distance.publish(pid_distance.compute_pid(tamanho_atual))

        # Controle PID parra corrigir dx
        pub_pid_dx.publish(pid_dx.compute_pid(dx))

if __name__ == '__main__':
    rospy.init_node("hover_navigate")

    # Instancia os PIDs

    pid_distance = PIDController(
        kp=               0.1, 
        ki=               0.1, 
        kd=               0.1,
        setpoint=         tamanho_capturado,
        lower_limit=      0.0,   
        upper_limit=      1.0    
    )
        
    pid_dx = PIDController(
        kp=               0.1, 
        ki=               0.1, 
        kd=               0.1,
        setpoint=         0,                # Disância do centro da tela em dx
        lower_limit=     -1.0,   
        upper_limit=      1.0    
    )

    # Rever a lógica de controle angular/giro depois.

    pid_angular_giro = PIDController(
        kp=               0.1, 
        ki=               0.1, 
        kd=               0.1,
        setpoint=         3,                # Velocidade angular desejada
        lower_limit=     -1.0,   
        upper_limit=      1.0    
    )

    # Publishers
    pub_linear = rospy.Publisher("/speed/velocidade_linear", Float32, queue_size=1)
    pub_angular = rospy.Publisher("/speed/velocidade_angular", Float32, queue_size=1)

    pub_pid_distance = rospy.Publisher("/pid/distance", Float32, queue_size=1)
    pub_pid_dx = rospy.Publisher("/pid/dx", Float32, queue_size=1)
    pub_pid_angular = rospy.Publisher("/pid/angular", Float32, queue_size=1)

    # Subscribers
    rospy.Subscriber("/dados_da_camera/distancia_x", Float32, callback_dx)
    rospy.Subscriber("/dados_da_camera/tamanho_atual", Float32, callback_tamanho)

    # Timer periódico (busca quando não há dados obtidos no subscriber -> sem objeto na tela)
    rospy.Timer(rospy.Duration(0.1), controle_periodico)

    rospy.spin()