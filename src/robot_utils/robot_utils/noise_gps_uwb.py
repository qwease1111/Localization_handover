import numpy as np
from filterpy.kalman import ExtendedKalmanFilter
import time
import json
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance

class Struct():
    init_flag = False
    
    max_noise_stddev = 0.3
    uwb_noise_stddev = 0.15
    gps_noise_stddev = 0.15
    
    max_covariance = 100
    uwb_covariance = 50.0
    gps_covariance = 5.0
    
    uwb_signal_strength = 50
    gps_signal_strength = 50
    
    robot_area = "A"

class noise_gps_uwb(Node):
    def __init__(self, node_name) :
        super().__init__(node_name)
        
        self.uwb_sub = self.create_subscription(Odometry,"/odometry/uwb",self.uwb_sub_callback,10)
        self.gps_sub = self.create_subscription(Odometry,"/odometry/gps",self.gps_sub_callback,10) ; self.gps_odometry_ = Odometry()
        self.wheel_odom_sub = self.create_subscription(Odometry,"/wheel/odometry",self.wheel_odom_sub_callback,10) ; self.wheel_odometry_ = Odometry()

        self.uwb_odometry_pub = self.create_publisher(Odometry,"odometry/uwb_noise",10) ; self.uwb_noise_odometry_ = Odometry()
        self.gps_odometry_pub = self.create_publisher(Odometry,"odometry/gps_noise",10) ; self.gps_noise_odometry_ = Odometry() 
        

        self.robot_status_pub = self.create_publisher(String,"robot_status",10) ; self.robot_status_str_ = String()

        self.struct_data = Struct()

        self.create_timer(0.1, self.robot_status_pub_func)

        self.get_logger().info("create %s ..." % node_name)
    
    
    def robot_status_pub_func(self):
        data = {
                "max_noise_stddev": self.struct_data.max_noise_stddev ,
                "uwb_noise_stddev": self.struct_data.uwb_noise_stddev ,
                "gps_noise_stddev": self.struct_data.gps_noise_stddev ,
                "max_covariance": self.struct_data.max_covariance ,
                "uwb_covariance": self.struct_data.uwb_covariance ,
                "gps_covariance": self.struct_data.gps_covariance ,
                "uwb_signal_strength": self.struct_data.uwb_signal_strength ,
                "gps_signal_strength": self.struct_data.gps_signal_strength ,
                "robot_area": self.struct_data.robot_area ,
                "robot_x" : round(self.wheel_odometry_.pose.pose.position.x,2) ,
                "robot_y" : round(self.wheel_odometry_.pose.pose.position.y,2)
            }
        
        json_string = json.dumps(data)
        self.robot_status_str_.data = json_string
        self.robot_status_pub.publish(self.robot_status_str_)

    
    def create_white_noise(self,stddev) :
        
        mean = 0
        
        white_noise = random.gauss(mean, stddev)
        return white_noise

    
    def uwb_sub_callback(self, data):
        self.uwb_noise_odometry_ = data
        self.uwb_noise_odometry_.pose.pose.position.x +=  self.create_white_noise(self.struct_data.uwb_noise_stddev)
        self.uwb_noise_odometry_.pose.pose.position.y +=  self.create_white_noise(self.struct_data.uwb_noise_stddev)

        self.uwb_noise_odometry_.pose.covariance[0] = self.struct_data.uwb_covariance  
        self.uwb_noise_odometry_.pose.covariance[7] = self.struct_data.uwb_covariance  

        if self.struct_data.init_flag == True :
            self.uwb_odometry_pub.publish(self.uwb_noise_odometry_)

    
    def gps_sub_callback(self, data):
        self.gps_odometry_ = data
        self.gps_noise_odometry_ = data
        self.gps_noise_odometry_.pose.pose.position.x +=  self.create_white_noise(self.struct_data.gps_noise_stddev)
        self.gps_noise_odometry_.pose.pose.position.y +=  self.create_white_noise(self.struct_data.gps_noise_stddev)

        self.gps_noise_odometry_.pose.covariance[0] = self.struct_data.gps_covariance  
        self.gps_noise_odometry_.pose.covariance[7] = self.struct_data.gps_covariance  

        if self.struct_data.init_flag == True :
            self.gps_odometry_pub.publish(self.gps_noise_odometry_)
    
    
    def wheel_odom_sub_callback(self, data):
        self.wheel_odometry_ = data
        x = self.wheel_odometry_.pose.pose.position.x
        y = self.wheel_odometry_.pose.pose.position.y
        
        if 0 <= x <= 7 and 0 <= y <= 7 :
            self.struct_data.robot_area = "D: Indoor Area"

            self.struct_data.gps_signal_strength = 0.0
            self.struct_data.gps_covariance = ((100.0 - self.struct_data.gps_signal_strength)/100)*self.struct_data.max_covariance
            self.struct_data.gps_noise_stddev = ((100.0 - self.struct_data.gps_signal_strength)/100)*self.struct_data.max_noise_stddev

            self.struct_data.uwb_signal_strength = 100.0
            self.struct_data.uwb_covariance = ((100.0 - self.struct_data.uwb_signal_strength)/100)*self.struct_data.max_covariance
            self.struct_data.uwb_noise_stddev = ((100.0 - self.struct_data.uwb_signal_strength)/100)*self.struct_data.max_noise_stddev

        
        
        elif 5.5 <= y <= 6.5 and 7 <= x <= (-0.163*(y**2) + 1.143*y + 7.000):
            self.struct_data.robot_area = "C: Handover Area(UWB denied)"

            self.struct_data.gps_signal_strength = 100 * (x - 7) / ((-0.163*(y**2) + 1.143*y + 7.000) - 7 )
            self.struct_data.gps_covariance = ((100.0 - self.struct_data.gps_signal_strength)/100)*self.struct_data.max_covariance
            self.struct_data.gps_noise_stddev = ((100.0 - self.struct_data.gps_signal_strength)/100)*self.struct_data.max_noise_stddev

            self.struct_data.uwb_signal_strength = 0.0
            self.struct_data.uwb_covariance = ((100.0 - self.struct_data.uwb_signal_strength)/100)*self.struct_data.max_covariance
            self.struct_data.uwb_noise_stddev = ((100.0 - self.struct_data.uwb_signal_strength)/100)*self.struct_data.max_noise_stddev

        
        elif ( 0 < y < 5.5 or 6.5 < y < 7 ) and 7 <= x <= (-0.163*(y**2) + 1.143*y + 7.000):
            self.struct_data.robot_area = "B: Handover Area"

            self.struct_data.gps_signal_strength = 100 * (x - 7) / ((-0.163*(y**2) + 1.143*y + 7.000) - 7 )
            self.struct_data.gps_covariance = ((100.0 - self.struct_data.gps_signal_strength)/100)*self.struct_data.max_covariance
            self.struct_data.gps_noise_stddev = ((100.0 - self.struct_data.gps_signal_strength)/100)*self.struct_data.max_noise_stddev

            self.struct_data.uwb_signal_strength = 50.0
            self.struct_data.uwb_covariance = ((100.0 - self.struct_data.uwb_signal_strength)/100)*self.struct_data.max_covariance
            self.struct_data.uwb_noise_stddev = ((100.0 - self.struct_data.uwb_signal_strength)/100)*self.struct_data.max_noise_stddev
        
        
        else :
            self.struct_data.robot_area = "A: Outdoor Area"

            self.struct_data.gps_signal_strength = 100.0
            self.struct_data.gps_covariance = ((100.0 - self.struct_data.gps_signal_strength)/100)*self.struct_data.max_covariance
            self.struct_data.gps_noise_stddev = ((100.0 - self.struct_data.gps_signal_strength)/100)*self.struct_data.max_noise_stddev

            self.struct_data.uwb_signal_strength = 0.0
            self.struct_data.uwb_covariance = ((100.0 - self.struct_data.uwb_signal_strength)/100)*self.struct_data.max_covariance
            self.struct_data.uwb_noise_stddev = ((100.0 - self.struct_data.uwb_signal_strength)/100)*self.struct_data.max_noise_stddev

        self.struct_data.init_flag = True
        


def main(args=None):
    rclpy.init(args=args)			  
    noise_gps_uwb_node = noise_gps_uwb("noise_gps_uwb_node") 
    rclpy.spin(noise_gps_uwb_node)       
    rclpy.shutdown()

if __name__ == "__main__":
    main()