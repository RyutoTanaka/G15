#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Float32

import RPi.GPIO as GPIO
import math

class MotorDriver(Node):
    PWM_RIGHT_PIN = 32
    PWM_LEFT_PIN = 33
    DIRECTION_RIGHT_PIN = 31
    DIRECTION_LEFT_PIN = 29

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PWM_RIGHT_PIN, GPIO.OUT, initial=GPIO.HIGH)
    right_pwm = GPIO.PWM(PWM_RIGHT_PIN, 1000)
    GPIO.setup(PWM_LEFT_PIN, GPIO.OUT, initial=GPIO.HIGH)
    left_pwm = GPIO.PWM(PWM_LEFT_PIN, 1000)
    GPIO.setup(DIRECTION_RIGHT_PIN, GPIO.OUT)
    GPIO.setup(DIRECTION_LEFT_PIN, GPIO.OUT)
    def __init__(self):
        super().__init__('motor_driber')
        
        # pwm_right (-1.0 ~ 1.0)
        self.pwm_right_subscription = self.create_subscription(
            Float32,
            'pwm_right',
            self.pwm_right_callback,
            qos.qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        
        # pwm_left (-1.0 ~ 1.0)
        self.pwm_left_subscription = self.create_subscription(
            Float32,
            'pwm_left',
            self.pwm_left_callback,
            qos.qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def __del__(self):
        right_pwm.stop()
        left_pwm.stop()
        GPIO.cleanup()
    
    def pwm_right_callback(self, msg):
        if msg.data >= 0.0 :
            GPIO.output(DIRECTION_RIGHT_PIN, GPIO.LOW)
        else :
            GPIO.output(DIRECTION_RIGHT_PIN, GPIO.HIGH)
        right_pwm.start(math.fabs(msg.data))


    def pwm_left_callback(self, msg):
        if msg.data >= 0.0 :
            GPIO.output(DIRECTION_LEFT_PIN, GPIO.HIGH)
        else :
            GPIO.output(DIRECTION_LEFT_PIN, GPIO.LOW)
        right_pwm.start(math.fabs(msg.data))


def main(args=None):
    rclpy.init(args=args)

    motor_driver = MotorDriver()

    rclpy.spin(motor_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
