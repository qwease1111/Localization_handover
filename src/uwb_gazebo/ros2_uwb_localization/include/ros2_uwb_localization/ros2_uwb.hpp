#ifndef ROS2_UWB_LOCALIZATION__ROS2_UWB_HPP_
#define ROS2_UWB_LOCALIZATION__ROS2_UWB_HPP_

#include <stdio.h>
#include <string.h>
#include <math.h>
//#include <stdlib.h>
#include <math.h>
#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rosmsgs/msg/ranging_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>

using namespace std::chrono_literals ;
using std::placeholders::_1 ;

class Ros2UwbLocalization : public rclcpp::Node {
    public:
        Ros2UwbLocalization();
        ~Ros2UwbLocalization();

        struct point_t {
            double x, y;
        };
        struct circle_t {
            struct point_t center;
            double r;
        };
        struct point {
            int32_t id ;
            double x, y ;
        };
    private:
        bool isInitialized ;                    // 基站的位置是否已经初始化得到
        bool tags_tf2_enable ;                  // 发布 tags tf2 变换
        bool anchors_tf2_enable ;               // 发布 anchors tf2 变换
        bool debug_enable ;                     // true if debug
        std::string odom_topic ;                // odom 发布话题名称
        std::string uwb_anchors_topic ;         // uwb_anchors 发布话题名称
        std::string uwb_ranging_topic ;         // uwb_ranging 发布话题名称

        struct point uwb_anchor[4] ;            // uwb anchor 基站位置
        double tag_uwb_ranges[4] ;              // tag-anchor 标签距离

        nav_msgs::msg::Odometry uwb_odom_ ;    // uwb 解算出来的 odom 里程计

        rclcpp::Subscription<rosmsgs::msg::RangingArray>::SharedPtr uwb_subscription_ ;                 // uwb 距离订阅
        rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr  anchor_subscription_;    // anchors 基站订阅

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr uwb_odome_pub_ ;
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_ ;

        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_ ;           // tf2 基站 anchor-map 静态发布
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;                         // tf2 标签 tag-map 动态变换

        void init_parameters();

        // uwb 三边法定位解算
        int double_equals(double a, double b) ;
        double distance_sqr(struct point_t* a, struct point_t* b) ;
        double distance(struct point_t* a, struct point_t* b) ;
        int insect(struct circle_t circles[], struct point_t points[]) ;
        void Cross_Point(struct circle_t circles[], struct point_t Location[]) ; 
        float norm(struct point p) ; // get the norm of a vector 求向量的范数
        void trilateration_xy(struct point point1, struct point point2, struct point point3, double r1, double r2, double r3) ;

        // ros2 话题订阅回调函数
        void uwbCallback(const rosmsgs::msg::RangingArray::SharedPtr msg);
        void anchorCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
};


#endif  // ROS2_UWB_LOCALIZATION__ROS2_UWB_HPP_