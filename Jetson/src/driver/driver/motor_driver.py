#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Float32

import Jetson.GPIO as GPIO
import math

class MotorDriver(Node):
    PWM_RIGHT_PIN = 33
    PWM_LEFT_PIN = 32
    DIRECTION_RIGHT_PIN = 16
    DIRECTION_LEFT_PIN = 12
    def __init__(self):
        super().__init__('motor_driber')
        
        # pwm_right (-1.0 ~ 1.0)
        self.pwm_right_subscription = self.create_subscription(
            Float32,
            'pwm_right',
            self.pwm_right_callback,
            qos.qos_profile_sensor_data)
        self.pwm_right_subscription  # prevent unused variable warning
                
        # pwm_left (-1.0 ~ 1.0)
        self.pwm_left_subscription = self.create_subscription(
            Float32,
            'pwm_left',
            self.pwm_left_callback,
            qos.qos_profile_sensor_data)
        self.pwm_left_subscription  # prevent unused variable warning
        
        timer_period = 0.1  # seconds        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timeout_cnt = 0
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.PWM_RIGHT_PIN, GPIO.OUT, initial=GPIO.HIGH)
        self.right_pwm = GPIO.PWM(self.PWM_RIGHT_PIN, 1000)
        GPIO.setup(self.PWM_LEFT_PIN, GPIO.OUT, initial=GPIO.HIGH)
        self.left_pwm = GPIO.PWM(self.PWM_LEFT_PIN, 1000)
        GPIO.setup(self.DIRECTION_RIGHT_PIN, GPIO.OUT)
        GPIO.setup(self.DIRECTION_LEFT_PIN, GPIO.OUT)
    
    def timer_callback(self):
        if(self.timeout_cnt > 40):
            self.right_pwm.stop()
            self.left_pwm.stop()
        elif(self.timeout_cnt > 20):
            self.right_pwm.start(1.0)
            self.left_pwm.start(1.0)
        self.timeout_cnt += 1
    
    def pwm_right_callback(self, msg):
        self.timeout_cnt = 0
        self.get_logger().info('I heard right: "%f"' % msg.data)
        if msg.data >= 0.0 :
            GPIO.output(self.DIRECTION_RIGHT_PIN, GPIO.LOW)
        else :
            GPIO.output(self.DIRECTION_RIGHT_PIN, GPIO.HIGH)
        self.right_pwm.start(math.fabs(msg.data))


    def pwm_left_callback(self, msg):
        self.timeout_cnt = 0
        self.get_logger().info('I heard left: "%f"' % msg.data)
        if msg.data > 0.0 :
            GPIO.output(self.DIRECTION_LEFT_PIN, GPIO.HIGH)
        else :
            GPIO.output(self.DIRECTION_LEFT_PIN, GPIO.LOW)
        self.left_pwm.start(math.fabs(msg.data))


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
