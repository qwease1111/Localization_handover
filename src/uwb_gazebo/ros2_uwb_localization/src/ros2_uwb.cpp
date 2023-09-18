#include "ros2_uwb_localization/ros2_uwb.hpp"

Ros2UwbLocalization::Ros2UwbLocalization()
: Node("ros2_uwb_localization_node") {
	// 初始化
	init_parameters();
	// 初始化 ROS2 Sub 和 Pub
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
	// auto custom_qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(1));

	// tf2 
	tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
	tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

	// Publish
	uwb_odome_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)));
	
	// Subscription
	anchor_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
		uwb_anchors_topic, qos, std::bind(&Ros2UwbLocalization::anchorCallback, this, _1));

	uwb_subscription_ = this->create_subscription<rosmsgs::msg::RangingArray>(
		uwb_ranging_topic, qos, std::bind(&Ros2UwbLocalization::uwbCallback, this, _1));

	RCLCPP_INFO(this->get_logger(), "ros2 uwb localization node has been initialised !");
}

Ros2UwbLocalization::~Ros2UwbLocalization()
{
	RCLCPP_INFO(this->get_logger(), "ros2 uwb localization node has been terminated");
}

void Ros2UwbLocalization::uwbCallback(const rosmsgs::msg::RangingArray::SharedPtr msg) {
	if (isInitialized == false || msg->ranging.size() < 3) {
		return;
	}
	// std::cout << "Size of the list : " << msg->ranging.size() << std::endl;
	for(int i = 0; i < msg->ranging.size(); i++) {
		// 匹配 id 不一直，即数据有缺失报警，并且不发布 odomtry/uwb
		if(std::to_string(uwb_anchor[i].id) != msg->ranging[i].anchor_id) {
			// RCLCPP_ERROR(this->get_logger(), " 无法获得 anchor%s 基站的数据 ..." , std::to_string(uwb_anchor[i].id).c_str() ) ;
			return;
		}
		// 获取每个 tag-anchor 的距离， 毫米mm -> 米m
		tag_uwb_ranges[i] = msg->ranging[i].range / 1000.0 ;
		// std::cout << i << " - " << uwb_anchor[i].id <<  " - " << std::to_string(uwb_anchor[i].id) << std::endl;
	}
	// 只用三个基站 uwb_anchor
	trilateration_xy(uwb_anchor[0], uwb_anchor[1], uwb_anchor[2], tag_uwb_ranges[0], tag_uwb_ranges[1], tag_uwb_ranges[2]);
}

void Ros2UwbLocalization::anchorCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
	if (isInitialized == false) {
		// 至少三个基站
		if ( msg->markers.size() < 3 ) {
			RCLCPP_ERROR(this->get_logger(), " uwb 基站不足3个 ... ");
			return ;
		}
		// 初始化获得所有基站的位置坐标
		for(int i = 0; i < msg->markers.size(); i++) {
			uwb_anchor[i].x = msg->markers[i].pose.position.x ;
			uwb_anchor[i].y = msg->markers[i].pose.position.y ;
			uwb_anchor[i].id = msg->markers[i].id ;
			// std::cout << uwb_anchor[i].id <<  " - " << std::to_string(uwb_anchor[i].id) << std::endl;
			if (anchors_tf2_enable == true) {
				geometry_msgs::msg::TransformStamped t;
				t.header.stamp = this->get_clock()->now();
				t.header.frame_id = "map" ;
				t.child_frame_id = "anchor" + std::to_string(msg->markers[i].id) ;
				// std::cout << t.child_frame_id << std::endl;
				t.transform.translation.x = uwb_anchor[i].x ;
				t.transform.translation.y = uwb_anchor[i].y ;
				tf_static_broadcaster_->sendTransform(t) ;
			}
		}
		// 初始化完成
		isInitialized = true ;
		RCLCPP_INFO(this->get_logger(), "Anchors pose has been initialised !");
	}
	// return;
}

void Ros2UwbLocalization::init_parameters() {
	isInitialized = false ;
	
	this->declare_parameter<bool>("debug_enable", false) ;	
	this->declare_parameter<bool>("tags_tf2_enable", false) ;	
	this->declare_parameter<bool>("anchors_tf2_enable", false) ;	
	this->declare_parameter<std::string>("odom_topic", "odometry/uwb") ;
	this->declare_parameter<std::string>("uwb_anchors_topic", "/uwb_anchors") ;
	this->declare_parameter<std::string>("uwb_ranging_topic", "/uwb_ranging") ;
	
	this->get_parameter("debug_enable", debug_enable) ;
	this->get_parameter("tags_tf2_enable", tags_tf2_enable) ;
	this->get_parameter("anchors_tf2_enable", anchors_tf2_enable) ;
	this->get_parameter("odom_topic", odom_topic) ;
	this->get_parameter("uwb_anchors_topic", uwb_anchors_topic) ;
	this->get_parameter("uwb_ranging_topic", uwb_ranging_topic) ;

	// debug 测试
	if ( debug_enable == true ) {
		uwb_anchor[0].x= 6.8;
		uwb_anchor[0].y = 6.8;
		tag_uwb_ranges[0] = 4.954; 	// 基站 1 距离 tag 标签的距离
		uwb_anchor[1].x = 6.8;
		uwb_anchor[1].y = 0.5;
		tag_uwb_ranges[1] = 4.429;	// 基站 2 距离 tag 标签的距离
		uwb_anchor[2].x = 0.5;
		uwb_anchor[2].y = 6.8;
		tag_uwb_ranges[2] = 4.424;	// 基站 3 距离 tag 标签的距离
		// 计算标签的位置坐标
		trilateration_xy(uwb_anchor[0], uwb_anchor[1], uwb_anchor[2], tag_uwb_ranges[0], tag_uwb_ranges[1], tag_uwb_ranges[2]);
	}
}

int Ros2UwbLocalization::double_equals(double a, double b) {
	static const double ZERO = 1e-9;
	return fabs(a - b) < ZERO;
}

double Ros2UwbLocalization::distance_sqr(struct point_t* a, struct point_t* b)
{
	return (a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y);
}

double Ros2UwbLocalization::distance(struct point_t* a, struct point_t* b)
{
	return sqrt(distance_sqr(a, b));
}

int Ros2UwbLocalization::insect(struct circle_t circles[], struct point_t points[])
{
	double d, a, b, c, p, q, r;
	double cos_value[2], sin_value[2];
	if (double_equals(circles[0].center.x, circles[1].center.x)
		&& double_equals(circles[0].center.y, circles[1].center.y)
		&& double_equals(circles[0].r, circles[1].r))
	{
		return -1;
	}

	d = distance(&circles[0].center, &circles[1].center);
	if (d > circles[0].r + circles[1].r
		|| d < fabs(circles[0].r - circles[1].r))
	{
		return 0;
	}

	a = 2.0 * circles[0].r * (circles[0].center.x - circles[1].center.x);
	b = 2.0 * circles[0].r * (circles[0].center.y - circles[1].center.y);
	c = circles[1].r * circles[1].r - circles[0].r * circles[0].r
		- distance_sqr(&circles[0].center, &circles[1].center);
	p = a * a + b * b;
	q = -2.0 * a * c;
	if (double_equals(d, circles[0].r + circles[1].r)
		|| double_equals(d, fabs(circles[0].r - circles[1].r)))
	{
		cos_value[0] = -q / p / 2.0;
		sin_value[0] = sqrt(1 - cos_value[0] * cos_value[0]);

		points[0].x = circles[0].r * cos_value[0] + circles[0].center.x;
		points[0].y = circles[0].r * sin_value[0] + circles[0].center.y;

		if (!double_equals(distance_sqr(&points[0], &circles[1].center),
			circles[1].r * circles[1].r))
		{
			points[0].y = circles[0].center.y - circles[0].r * sin_value[0];
		}
		return 1;
	}

	r = c * c - b * b;
	cos_value[0] = (sqrt(q * q - 4.0 * p * r) - q) / p / 2.0;
	cos_value[1] = (-sqrt(q * q - 4.0 * p * r) - q) / p / 2.0;
	sin_value[0] = sqrt(1 - cos_value[0] * cos_value[0]);
	sin_value[1] = sqrt(1 - cos_value[1] * cos_value[1]);

	points[0].x = circles[0].r * cos_value[0] + circles[0].center.x;
	points[1].x = circles[0].r * cos_value[1] + circles[0].center.x;
	points[0].y = circles[0].r * sin_value[0] + circles[0].center.y;
	points[1].y = circles[0].r * sin_value[1] + circles[0].center.y;

	if (!double_equals(distance_sqr(&points[0], &circles[1].center),
		circles[1].r * circles[1].r))
	{
		points[0].y = circles[0].center.y - circles[0].r * sin_value[0];
	}
	if (!double_equals(distance_sqr(&points[1], &circles[1].center),
		circles[1].r * circles[1].r))
	{
		points[1].y = circles[0].center.y - circles[0].r * sin_value[1];
	}
	if (double_equals(points[0].y, points[1].y)
		&& double_equals(points[0].x, points[1].x))
	{
		if (points[0].y > 0)
		{
			points[1].y = -points[1].y;
		}
		else
		{
			points[0].y = -points[0].y;
		}
	}
	return 2;
}

void Ros2UwbLocalization::Cross_Point(struct circle_t circles[], struct point_t Location[])
{
	int cross_num = 5;
	struct point_t cross_points[2];
	cross_num = insect(circles, cross_points); // 0 1

	if (cross_num == 2)
	{
		double points_AC_0 = distance(&cross_points[0], &circles[2].center);
		double points_AC_1 = distance(&cross_points[1], &circles[2].center);

		if (abs((int)(points_AC_0 - circles[2].r) )< abs((int)(points_AC_1 - circles[2].r)))//cross_point[0]
		{
			Location[0].x = cross_points[0].x;
			Location[0].y = cross_points[0].y;
		}
		else
		{
			Location[0].x = cross_points[1].x;
			Location[0].y = cross_points[1].y;
		}

	}
	else if (cross_num == 1 || cross_num == 0)
	{
		Location[0].x = cross_points[0].x;
		Location[0].y = cross_points[0].y;
	}
}

float Ros2UwbLocalization::norm(struct point p) // get the norm of a vector 求向量的范数
{
	return pow(pow(p.x, 2) + pow(p.y, 2), .5);
}

// 三边定位求解坐标
void Ros2UwbLocalization::trilateration_xy(struct point point1, struct point point2, struct point point3, double r1, double r2, double r3) {
	struct point resultPose;
	// unit vector in a direction from point1 to point 2  从点1到点2方向上的单位向量
	double p2p1Distance = pow(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2), 0.5);
	struct point ex = { -1 , (point2.x - point1.x) / p2p1Distance, (point2.y - point1.y) / p2p1Distance };
	struct point aux = { -1 , point3.x - point1.x,point3.y - point1.y };
	// signed magnitude of the x component  x分量的符号大小
	double i = ex.x * aux.x + ex.y * aux.y;
	// the unit vector in the y direction.  y方向的单位向量。T
	struct point aux2 = { -1 , point3.x - point1.x - i * ex.x, point3.y - point1.y - i * ex.y };
	struct point ey = { -1 , aux2.x / norm(aux2), aux2.y / norm(aux2) };
	// the signed magnitude of the y component  y分量的符号大小
	double j = ey.x * aux.x + ey.y * aux.y;
	// coordinates  协调
	double x = (pow(r1, 2) - pow(r2, 2) + pow(p2p1Distance, 2)) / (2 * p2p1Distance);
	double y = (pow(r1, 2) - pow(r3, 2) + pow(i, 2) + pow(j, 2)) / (2 * j) - i * x / j;
	// result coordinates   结果坐标
	double finalX = point1.x + x * ex.x + y * ey.x;
	double finalY = point1.y + x * ex.y + y * ey.y;
	resultPose.x = finalX;
	resultPose.y = finalY;

	// 对 ros2 uwb_odom_ 数据更新
	if( isInitialized == true ) {
		uwb_odom_.header.stamp = this->now() ;
		uwb_odom_.header.frame_id = "map" ;
		uwb_odom_.pose.pose.position.x = resultPose.x + 0.185 ;
		uwb_odom_.pose.pose.position.y = resultPose.y - 0.163 ;
		uwb_odome_pub_->publish(uwb_odom_) ;
		// 是否进行 tf2 变换
		if (tags_tf2_enable == true) {
			geometry_msgs::msg::TransformStamped t ;
			t.header.stamp = this->get_clock()->now() ;
			t.header.frame_id = "map" ;
			t.child_frame_id = "tag" ;
			t.transform.translation.x = uwb_odom_.pose.pose.position.x ;
			t.transform.translation.y = uwb_odom_.pose.pose.position.y ;
			tf_broadcaster_->sendTransform(t) ;
		}
		return;
	}
	RCLCPP_INFO(this->get_logger(), "TAG TEST LOC : (%3.2f, %3.2f)", resultPose.x, resultPose.y) ;

}

/*************
** Main
**************/
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Ros2UwbLocalization>());
	rclcpp::shutdown();
	return 0;
}
