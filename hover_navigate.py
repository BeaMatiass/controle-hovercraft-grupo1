#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
from pid import controle_pid

# Objetos globais
pub_angular = None
pub_linear = None
pid_erro_linear = None
pid_erro_angular = None

# Variáveis globais
dx_normalizado = 0.0

angulo_atual = 0.0
angulo_rad = 3.1415 * 0.2   # ~36 graus

vel_angular_real = 0.0
vel_angular_max = 1.0    # determinar valor

vel_linear_real = 0.0
vel_linear_max = 0.5    # determinar valor

distancia_z = 0.0
distancia_z_max = 300.0    # determinar valor 

ultimo_tempo_visto = None

def callback_mavros_vel(msg):
    global vel_angular_real, vel_linear_real
    vel_angular_real = msg.twist.angular.z
    vel_linear_real = msg.twist.linear.x
    return

def callback_angulo(msg):
    global angulo_atual
    angulo_atual = msg.data
    return

def callback_z(msg):
    global distancia_z
    distancia_z = msg.data
    return

def callback_dx(msg):
    global ultimo_tempo_visto, dx_normalizado
    dx_normalizado = msg.data
    ultimo_tempo_visto = rospy.Time.now()
    return

# Talvez não seja necessário dado o PID (analisar isso com calma)
def saturar(valor, minimo, maximo):
    return max(min(valor, maximo), minimo) # recomendação do Chat, em razão de ruídos

def controle_periodico(_event):
    global ultimo_tempo_visto

    if ultimo_tempo_visto is None:
        return

    tempo_sem_objeto = (rospy.Time.now() - ultimo_tempo_visto).to_sec()

    # Alvo perdido
    if tempo_sem_objeto > 3.0:
        pub_linear.publish(0.0)
        pub_angular.publish(0.0) # valores arbitrários (depois corrigir)
        return

    # -- AÇÃO LINEAR --
    frac = distancia_z / distancia_z_max
    frac = saturar(frac, 0.0, 1.0)

    vel_linear_desejada = vel_linear_max * frac
    
    pid_erro_linear.setpoint = vel_linear_desejada
    comando_linear = pid_erro_linear.processar_pid(vel_linear_real)
    
    comando_linear = saturar(comando_linear, -1.0, 1.0)

    pub_linear.publish(comando_linear)

    # -- AÇÃO ANGULAR --
    erro_norm = saturar(angulo_atual / angulo_rad, -1.0, 1.0)
    vel_angular_desejada = vel_angular_max * erro_norm

    pid_erro_angular.setpoint = vel_angular_desejada
    comando_angular = pid_erro_angular.processar_pid(vel_angular_real)

    comando_angular = saturar(comando_angular, -1.0, 1.0)

    pub_angular.publish(comando_angular)

if __name__ == '__main__':
    rospy.init_node("hover_navigate")

    # mudei a lógica, agora não é mais tomando o "tamanho_capturado", mas a velocidade baseada na 
    # distância (consideraremos uma distancia mínima segura sendo o zero de um eixo e, então, 
    # pegaremos a distância máxima)

    pid_erro_linear = controle_pid(
        kp = 0.1, 
        ki = 0.1, 
        kd = 0.05,
        setpoint = 0.0, 
        lower_limit = -1.0,
        upper_limit = 1.0
    )
    
    pid_erro_angular = controle_pid(
        kp = 0.4,
        ki = 0.1,
        kd = 0.05,
        setpoint = 0.0,
        lower_limit = -1.0,
        upper_limit = 1.0 
    )

    # Publishers
    pub_linear = rospy.Publisher("/speed/acao_linear", Float32, queue_size=1)
    pub_angular = rospy.Publisher("/speed/acao_angular", Float32, queue_size=1)

    # Subscribers
    rospy.Subscriber("/dados_da_camera/distancia_x", Float32, callback_dx)
    rospy.Subscriber("/dados_da_camera/angulo_atual", Float32, callback_angulo)
    rospy.Subscriber("/dados_da_camera/distancia_z", Float32, callback_z)
    rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, callback_mavros_vel)

    # Timer periódico
    rospy.Timer(rospy.Duration(0.1), controle_periodico)

    rospy.spin()
