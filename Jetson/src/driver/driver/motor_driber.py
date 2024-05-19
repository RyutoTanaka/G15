#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Uint16

import RPi.GPIO as GPIO

class MotorDriver(Node):
    PWM_RIGHT_PIN = 32
    PWM_LEFT_PIN = 33

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(PWM_RIGHT_PIN, GPIO.OUT, initial=GPIO.HIGH)
    right_pwm = GPIO.PWM(PWM_RIGHT_PIN, 0xFFFF)
    GPIO.setup(PWM_LEFT_PIN, GPIO.OUT, initial=GPIO.HIGH)
    left_pwm = GPIO.PWM(PWM_LEFT_PIN, 0xFFFF)

    def __init__(self):
        super().__init__('motor_driber')
        self.pwm_right_subscription = self.create_subscription(
            Uint16,
            'pwm_right',
            self.pwm_right_callback,
            qos.qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        self.pwm_left_subscription = self.create_subscription(
            Uint16,
            'pwm_left',
            self.pwm_left_callback,
            qos.qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def __del__(self):
        right_pwm.stop()
        left_pwm.stop()
        GPIO.cleanup()
    
    def pwm_right_callback(self, msg):
        right_pwm.start(msg.data)

    def pwm_left_callback(self, msg):
        right_pwm.start(msg.data)


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
