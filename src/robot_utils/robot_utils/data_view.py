import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import matplotlib.pyplot as plt
import time
import json

class MultiVisualization(Node):
    def __init__(self):
        super().__init__('multi_visualization')
        self.fig, self.axs = plt.subplots(2, 2, figsize=(10, 10))
        
        self.uwb_ratio = 0.5            
        self.gps_ratio = 0.5            
        self.max_show = 100             
        self.area = None                
        self.real_trajectory = []       
        self.estimated_trajectory = []  
        self.position_error = []        
        self.estimated_trajectory_times = [] 
        self.real_trajectory_times = []      
        self.times = []
        self.time = 0
        self.dt = 0.1           
        
        self.create_subscription(Odometry,"/odometry/global",self.estimated_trajectory_sub,10)
        
        self.create_subscription(Odometry,"/odometry/gps",self.real_trajectory_sub,10)
        
        self.create_subscription(String,"robot_status",self.robot_status_sub,10)
        
        self.create_timer(0.01, self.robot_status_show)
    
    def robot_status_show(self):
        self.axs[0, 0].clear()
        self.axs[0, 1].clear()
        self.axs[1, 0].clear()
        self.axs[1, 1].clear()
        
        
        try :
            sensor_signal = [self.uwb_ratio, self.gps_ratio]
            labels = ['UWB', 'GPS']
            self.axs[0, 0].pie(sensor_signal, labels=labels, autopct='%1.1f%%')
            self.axs[0, 0].set_title('Sensor Signal Ratio')
        except :
                pass

        
        try :
            self.axs[0, 1].text(0.5, 0.5, self.area, fontsize=50, ha='center', va='center')
            self.axs[0, 1].axis('off')
            self.axs[0, 1].set_title('Indoor/Outdoor')
        except :
                pass

        
        try :
            real_x, real_y = zip(*self.real_trajectory)
            estimated_x, estimated_y = zip(*self.estimated_trajectory)
            self.axs[1, 0].plot(self.times, real_x, label='real-x')
            self.axs[1, 0].plot(self.times, real_y, label='real-y')
            self.axs[1, 0].plot(self.times, estimated_x, label='estimated-x')
            self.axs[1, 0].plot(self.times, estimated_y, label='estimated-y')
            self.axs[1, 0].legend()
            self.axs[1, 0].set_title('Trajectory')
        except :
                pass

        
        try :
            error_x, error_y = zip(*self.position_error)
            self.axs[1, 1].plot(self.times, error_x, label='error-x')
            self.axs[1, 1].plot(self.times, error_y, label='error-y')
            self.axs[1, 1].legend()
            self.axs[1, 1].set_title('Position Error')
        except :
            pass
        
        plt.tight_layout()
        
        
        plt.pause(0.01)


    def estimated_trajectory_sub(self, msgs):
        self.estimated_trajectory.append((msgs.pose.pose.position.x, msgs.pose.pose.position.y))
        if len(self.estimated_trajectory) > self.max_show :
            self.estimated_trajectory.pop(0)
        
        self.estimated_trajectory_times.append(msgs.header.stamp.sec)
        if len(self.estimated_trajectory_times) > self.max_show :
                self.estimated_trajectory_times.pop(0)

    def real_trajectory_sub(self, msgs):
        self.real_trajectory.append((msgs.pose.pose.position.x, msgs.pose.pose.position.y))
        if len(self.real_trajectory) > self.max_show :
            self.real_trajectory.pop(0)
        
        self.real_trajectory_times.append(msgs.header.stamp.sec)
        if len(self.real_trajectory_times) > self.max_show :
                self.real_trajectory_times.pop(0)
        
        self.time += self.dt
        self.times.append(self.time)
        if len(self.times) > self.max_show :
                self.times.pop(0)

        
        try :
            self.position_error.append((abs(self.estimated_trajectory[-1][0]-self.real_trajectory[-1][0]) ,
                                        abs(self.estimated_trajectory[-1][1]-self.real_trajectory[-1][1])))
            if len(self.position_error) > self.max_show :
                self.position_error.pop(0)
        except :
            pass

    def robot_status_sub(self, msgs):
        string_json = json.loads(msgs.data)
        self.area = string_json['robot_area']
        self.uwb_ratio = string_json['uwb_signal_strength'] / 100.0
        self.gps_ratio = string_json['gps_signal_strength'] / 100.0

        

def main(args=None):
    rclpy.init(args=args)
    multi_visualization_node = MultiVisualization()
    rclpy.spin(multi_visualization_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
