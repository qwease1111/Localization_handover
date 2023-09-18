import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import time

class MultiOdomVisualization(Node):
    def __init__(self):
        super().__init__('multi_odom_visualization')
        self.odom_subscriptions = {}
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Value')
        self.ax.set_title('Multi-Odometry Visualization')
        self.ax.grid(True)
        self.max_view = 100
        self.times = {}
        self.xs = {}
        self.ys = {}

    def subscribe_to_odom_topic(self, topic_name):
        self.odom_subscriptions[topic_name] = self.create_subscription(
            Odometry,
            topic_name,
            lambda msg, topic=topic_name: self.odom_callback(msg, topic),
            10)
        
        if topic_name not in self.times:
            self.times[topic_name] = []
            self.xs[topic_name] = []
            self.ys[topic_name] = []

    def odom_callback(self, msg, topic_name):
        # 获取odom数据中的x和y坐标
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # 获取当前时间戳
        current_time = time.time()

        # 检测数据间断
        if len(self.times[topic_name]) > 0 and current_time - self.times[topic_name][-1] > 1.0:
            # 如果时间间隔大于1秒，说明发生了间断
            self.times[topic_name] = []
            self.xs[topic_name] = []
            self.ys[topic_name] = []

        # 将数据添加到列表中
        self.times[topic_name].append(current_time)
        self.xs[topic_name].append(x)
        self.ys[topic_name].append(y)

        if len(self.times[topic_name]) > self.max_view :
            self.times[topic_name].pop(0)

        if len(self.xs[topic_name]) > self.max_view :
            self.xs[topic_name].pop(0)

        if len(self.ys[topic_name]) > self.max_view :
            self.ys[topic_name].pop(0)


        # 绘制x和y坐标
        self.ax.clear()
        for topic_name in self.times:
            self.ax.plot(self.times[topic_name], self.xs[topic_name], label=f'{topic_name} X')
            self.ax.plot(self.times[topic_name], self.ys[topic_name], label=f'{topic_name} Y')

        # 更新图形
        self.ax.legend()
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Value')
        self.ax.set_title('Multi-Odometry Visualization')
        self.ax.grid(True)
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    multi_odom_visualization_node = MultiOdomVisualization()
    # for topic_name in ['/odometry/uwb', '/odometry/gps', '/odometry/global', '/odometry/ekf']:  # 替换为你的实际odom话题名字
    for topic_name in ['/odometry/uwb_noise', '/odometry/gps_noise', '/odometry/global']:  # 替换为你的实际odom话题名字
        multi_odom_visualization_node.subscribe_to_odom_topic(topic_name)
    rclpy.spin(multi_odom_visualization_node)
    multi_odom_visualization_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
