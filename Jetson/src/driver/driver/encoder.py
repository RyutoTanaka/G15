import rclpy
from rclpy.node import Node
from rclpy import qos
from sensor_msgs.msg import JointState
from datetime import datetime
import Jetson.GPIO as GPIO
import math

class EncoderPublisher(Node):
    # エンコーダーの分解能
    RESOLUTION = 2048
    # エンコーダーが1回転するときにタイヤは何回転するか
    ENC_TO_TIRE_L = 10/28
    ENC_TO_TIRE_R = -10/28
    # GPIOpin番号
    L_A_PIN = 1
    L_B_PIN = 2
    L_Z_PIN = 3
    R_A_PIN = 4
    R_B_PIN = 5
    R_Z_PIN = 6
    
    def __init__(self):
        super().__init__('encoder')
        self.tire_state_publisher_ = self.create_publisher(JointState, 'tire_state', qos.qos_profile_sensor_data)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.tire_state_callback)
        # Tireオブジェクトを定義
        self.l_tire = Tire("left")
        self.r_tire = Tire("right")
        # Encoderオブジェクトを定義
        self.l_enc = Encoder(self.L_A_PIN,self.L_B_PIN,self.L_Z_PIN)
        self.r_enc = Encoder(self.R_A_PIN,self.R_B_PIN,self.R_Z_PIN)
        self.last_l_enc_cnt = 0
        self.last_r_enc_cnt = 0
        
        # GPIOの設定
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.L_A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.L_B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.L_Z_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.R_A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.R_B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.R_Z_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # GPIO割り込みの設定
        GPIO.add_event_detect(self.L_A_PIN, GPIO.BOTH, callback=self.l_enc.a_callback)
        GPIO.add_event_detect(self.L_B_PIN, GPIO.BOTH, callback=self.l_enc.b_callback)
        GPIO.add_event_detect(self.L_Z_PIN, GPIO.BOTH, callback=self.l_enc.z_callback)
        GPIO.add_event_detect(self.R_A_PIN, GPIO.BOTH, callback=self.r_enc.a_callback)
        GPIO.add_event_detect(self.R_B_PIN, GPIO.BOTH, callback=self.r_enc.b_callback)
        GPIO.add_event_detect(self.R_Z_PIN, GPIO.BOTH, callback=self.r_enc.z_callback)

    def __del__(self):
        GPIO.cleanup()
        
    def get_tire_state(self):
        l_z_flag = self.l_enc.z_flag
        r_z_flag = self.r_enc.z_flag
        l_cnt = self.l_enc.cnt()
        r_cnt = self.r_enc.cnt()
        # カウンタ境目の処理
        if(l_z_flag):
            if self.last_l_enc_cnt >= 0 :
                # 2047 → 1 ならば 2047 → 2049
                l_cnt += self.RESOLUTION
            else:
                # -2047 → 1 ならば -2047 → -2047
                l_cnt -= self.RESOLUTION

        if(r_z_flag):
            if self.last_r_enc_cnt >= 0 :
                # 2047 → 1 ならば 2047 → 2049
                r_cnt += self.RESOLUTION
            else:
                # -2047 → 1 ならば -2047 → -2047
                r_cnt -= self.RESOLUTION

        # 前回との位置のカウント差分
        l_cnt_diff = l_cnt - self.last_l_enc_cnt
        r_cnt_diff = r_cnt - self.last_r_enc_cnt
        # カウント差分をタイヤのラジアンに変換
        l_pos_diff = 2*math.pi* self.ENC_TO_TIRE_L * (l_cnt_diff/self.RESOLUTION)
        r_pos_diff = 2*math.pi* self.ENC_TO_TIRE_R * (r_cnt_diff/self.RESOLUTION)
        # posをセットする
        self.l_tire.set_pos(datetime.now(), self.l_tire.pos + l_pos_diff)
        self.r_tire.set_pos(datetime.now(), self.r_tire.pos + r_pos_diff)
        self.last_l_enc_cnt = l_cnt
        self.last_r_enc_cnt = r_cnt
        

    def tire_state_callback(self):
        self.get_tire_state()
        msg = JointState()
        msg.header.stamp = datetime.now().to_msg()
        msg.name = [self.l_tire.name,self.r_tire.name]
        msg.position = [self.l_tire.pos,self.r_tire.pos]
        msg.velocity = [self.l_tire.vel,self.r_tire.vel]
        self.left_tire_state_publisher_.publish(msg)
        self.get_logger().info('TireVel left:%f right:%f' % msg.vel[0] % msg.vel[1])


class Encoder:
    def __init__(self,a_pin,b_pin,z_pin):
        # 回転に伴って1ずつ増減するカウンター
        self.cnt = 0
        # 前回のA相の値
        self.last_a = 0
        # 前回のB相の値
        self.last_b = 0
        # １回転してZ相が上がったときのフラグ
        self.z_flag = False
        # 各相のGPIOのピン番号
        self.A_PIN = a_pin
        self.B_PIN = b_pin
        self.Z_PIN = z_pin
    
    # 各相の割り込み関数
    def a_callback(self):
        a = GPIO.input(self.A_PIN)
        b = GPIO.input(self.B_PIN)
        if a != self.last_a:
            if a == b:
                self.cnt += 1
            else:
                self.cnt -= 1
            self.last_a = a
        
    def b_callback(self):
        a = GPIO.input(self.A_PIN)
        b = GPIO.input(self.B_PIN)
        if b != self.last_b:
            if b != a:
                self.cnt += 1
            else:
                self.cnt -= 1
            self.last_b = b
    
    def z_callback(self):
        if abs(self.cnt) >= 10:
            self.z_flag = True
        self.cnt = 0
        
    
    def cnt(self):
        self.z_flag = False
        return self.cnt

class Tire:
    def __init__(self,name):
        self.time = datetime.now()
        self.name = name
        self.pos = 0.0
        self.vel = 0.0
    def __init__(self,time,name,pos,vel):
        self.time = time        
        self.name = name
        self.pos = pos
        self.vel = vel
    
    # 現在時刻[s]とpos[rad]を代入して、状態を更新する。
    def set_pos(self,new_time,new_pos):
        self.vel = (new_pos-self.pos) / (new_time-self.time).total_seconds()
        self.pos = new_pos
        self.time = new_time


def main(args=None):
    rclpy.init(args=args)

    encoder_publisher = EncoderPublisher()

    rclpy.spin(encoder_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    encoder_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()