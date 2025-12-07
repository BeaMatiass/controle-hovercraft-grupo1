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

vel_angular_real = None
vel_angular_max = 1.0    # determinar valor

vel_linear_real = None
vel_linear_max = 0.5    # determinar valor

distancia_z = 0.0
distancia_z_max = 250.0    # 250 cm na vida real, usaremos 250 cm para garantir que ele vai parar corretamente 

ultimo_tempo_visto = None

def callback_mavros_vel(msg):
    global vel_angular_real, vel_linear_real
    vel_angular_real = msg.twist.angular.z
    vel_linear_real = msg.twist.linear.x
    return

def callback_angulo(msg):
    global angulo_atual, ultimo_tempo_visto
    angulo_atual = msg.data
    ultimo_tempo_visto = rospy.Time.now()
    return

def callback_z(msg):
    global distancia_z
    distancia_z = msg.data
    return

# Talvez não seja necessário dado o PID (analisar isso com calma)
def saturar(valor, minimo, maximo):
    return max(min(valor, maximo), minimo) # recomendação do Chat, em razão de ruídos

def controle_periodico(_event):
    global ultimo_tempo_visto

    if ultimo_tempo_visto is None:
        pub_linear.publish(0.1)
        pub_angular.publish(0.5) 
        return

    tempo_sem_objeto = (rospy.Time.now() - ultimo_tempo_visto).to_sec()

    # Alvo perdido
    if tempo_sem_objeto > 3.0:
        pub_linear.publish(0.1)
        pub_angular.publish(0.5) # valores arbitrários (depois corrigir)
        return

    # -- AÇÃO LINEAR --
    if distancia_z > 50:
        frac = distancia_z / distancia_z_max
        frac = saturar(frac, 0.0, 1.0)
    else:
        frac = 0
    
    vel_linear_desejada = vel_linear_max * frac
    
    pid_linear.setpoint = vel_linear_desejada
    comando_linear = pid_linear.processar_pid(vel_linear_real)
    
    comando_linear = saturar(comando_linear, 0.0, 1.0) # depois será convertido para PWM

    pub_linear.publish(comando_linear)

    # -- AÇÃO ANGULAR --
    erro_norm = saturar(angulo_atual / angulo_rad, -1.0, 1.0)
    vel_angular_desejada = vel_angular_max * erro_norm

    pid_angular.setpoint = vel_angular_desejada
    comando_angular = pid_angular.processar_pid(vel_angular_real)

    comando_angular = saturar(comando_angular, -1.0, 1.0)

    pub_angular.publish(comando_angular)

    # Printar (para tunagem)

    rospy.loginfo(f"[LIN] set={pid_linear.setpoint:.2f}  real={vel_linear_real:.2f}  cmd={comando_linear:.2f}")
    rospy.loginfo(f"[ANG] set={pid_angular.setpoint:.2f}  real={vel_angular_real:.2f}  cmd={comando_angular:.2f}")

if __name__ == '__main__':
    rospy.init_node("hover_navigate")

    pid_linear = controle_pid(
        kp = 1.0, # kp foi aumentado, visto que na simulação a resposta costuma precisar de mais ganho direto
        ki = 0.0, 
        kd = 0.0,
        setpoint = 0.0, 
        lower_limit = -1.0,
        upper_limit = 1.0
    )
    
    pid_angular = controle_pid(
        kp = 1.0,
        ki = 0.1,
        kd = 0.05,
        setpoint = 0.0,
        lower_limit = -1.0,
        upper_limit = 1.0 
    )

    # Publishers
    pub_linear = rospy.Publisher("/pid/distance", Float32, queue_size=1)
    pub_angular = rospy.Publisher("/pid/angular", Float32, queue_size=1)

    # Subscribers
    rospy.Subscriber("/dados_da_camera/angulo_atual", Float32, callback_angulo)
    rospy.Subscriber("/dados_da_camera/distancia_z", Float32, callback_z)
    rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, callback_mavros_vel) # verificar se esse é o nome do tópico mesmo

    # Timer periódico
    rospy.Timer(rospy.Duration(0.1), controle_periodico)

    rospy.spin()
