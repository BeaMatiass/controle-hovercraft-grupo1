#!/usr/bin/env python3
import time
import rospy
from std_msgs.msg import Float32
import pigpio

GPIO_PWM_THROTTLE = 12
GPIO_PWM_LIFT = 13
PERIODO = 0.02  # 20 ms

throttle = 0.0


def callback_velocidade_linear(msg):
    global throttle
    throttle = float(msg.data)


def calibrar(pi, pin):
    for _ in range(250):
        pi.write(pin, 1)
        time.sleep(0.002)
        pi.write(pin, 0)
        time.sleep(0.018)

    for _ in range(250):
        pi.write(pin, 1)
        time.sleep(0.001)
        pi.write(pin, 0)
        time.sleep(0.019)


def pwm_loop(pi, pub_pwm):
    global throttle

    while not rospy.is_shutdown():
        duty_throttle = 5.0 + (throttle * 0.05)
        duty_throttle = max(5.0, min(10.0, duty_throttle))

        duty_lift = 10.0

        high_throttle = (duty_throttle / 100.0) * PERIODO
        high_lift = (duty_lift / 100.0) * PERIODO

        t0 = time.time()
        end_throttle = t0 + high_throttle
        end_lift = t0 + high_lift
        end_period = t0 + PERIODO

        pi.write(GPIO_PWM_THROTTLE, 1)
        pi.write(GPIO_PWM_LIFT, 1)

        while True:
            now = time.time()

            if now >= end_throttle and pi.read(GPIO_PWM_THROTTLE) == 1:
                pi.write(GPIO_PWM_THROTTLE, 0)

            if now >= end_lift and pi.read(GPIO_PWM_LIFT) == 1:
                pi.write(GPIO_PWM_LIFT, 0)

            if now >= end_period:
                break

            time.sleep(0.0005)

        if pub_pwm is not None:
            pub_pwm.publish(duty_throttle)


def on_shutdown(pi):
    pi.write(GPIO_PWM_THROTTLE, 0)
    pi.write(GPIO_PWM_LIFT, 0)
    pi.stop()


if __name__ == "__main__":
    rospy.init_node("hover_pwm")

    pi = pigpio.pi()
    if not pi.connected:
        rospy.logerr("Nao foi possivel conectar ao pigpiod.")
        exit(1)

    pi.set_mode(GPIO_PWM_THROTTLE, pigpio.OUTPUT)
    pi.set_mode(GPIO_PWM_LIFT, pigpio.OUTPUT)

    calibrar(pi, GPIO_PWM_THROTTLE)
    calibrar(pi, GPIO_PWM_LIFT)

    pub_pwm = rospy.Publisher("/pwm/throttle", Float32, queue_size=1)

    rospy.Subscriber("/speed/velocidade_linear", Float32, callback_velocidade_linear, queue_size=1)

    rospy.on_shutdown(lambda: on_shutdown(pi))

    try:
        pwm_loop(pi, pub_pwm)
    except rospy.ROSInterruptException:
        pass