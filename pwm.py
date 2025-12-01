#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
import pigpio
import os
import sys

gpio_throttle = 12
gpio_lift = 13
gpio_servo = 18

pwm_value_dist = 0.0
pwm_value_servo = 0.0
pi = None

def clamp(value, lower, upper):
    return max(lower, min(value, upper))

def pwm_loop(_event):
    global pi

    if pi is None:
        return

    duty_cycle_throttle = int(clamp(pwm_value_dist, 0.0, 1.0) * 1000 + 1000)  # [0,1] -> [1000,2000]
    pi.set_servo_pulsewidth(gpio_throttle, duty_cycle_throttle)

    duty_cycle_servo = int(clamp(pwm_value_servo, -1.0, 1.0) * 400 + 1500)
    pi.set_servo_pulsewidth(gpio_servo, duty_cycle_servo)

if __name__ == "__main__":
    rospy.init_node("hover_pwm")

    os.system("pigpiod")

    pi = pigpio.pi()

    if not pi.connected:
        rospy.logerr("Não foi possível conectar ao daemon pigpiod.")
        sys.exit(1)

    def callback_pid_dist(msg):
        global pwm_value_dist
        pwm_value_dist = msg.data

    def callback_pid_servo(msg):
        global pwm_value_servo
        pwm_value_servo = msg.data

    rospy.Subscriber("/pid/throttle", Float32, callback_pid_dist, queue_size=1)
    rospy.Subscriber("/pid/servo", Float32, callback_pid_servo, queue_size=1)

    rospy.Timer(rospy.Duration(0.02), pwm_loop)

    rospy.on_shutdown(pi.stop)

    rospy.spin()