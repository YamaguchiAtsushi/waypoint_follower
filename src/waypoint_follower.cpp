#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>


#include <map>
#include <math.h>
#include <vector>
#include "std_msgs/Empty.h"
#include <time.h>
#include <fstream>
#include <sstream>
#include <string>


// オドメトリから得られる現在の位置と姿勢
double robot_x, robot_y;
double roll, pitch, yaw;

geometry_msgs::Quaternion robot_r;
geometry_msgs::Twist twist; // 指令する速度、角速度
std::map<std::string, double> params_; // パラメータをここに格納
geometry_msgs::PoseStamped goal; // 目標地点
sensor_msgs::LaserScan scan;
std::vector<geometry_msgs::PoseStamped> waypoints; // waypointを格納するvector

int flag = 0;
int scan_flag = 0;
int object_flag = 0;


// オドメトリのコールバック
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	robot_x = msg->pose.pose.position.x;
	robot_y = msg->pose.pose.position.y;
	robot_r = msg->pose.pose.orientation;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_)
{
    float nandeyanen = scan_->range_max;
    scan.header = scan_->header;
    scan.angle_min = scan_->angle_min;
    scan.angle_max = scan_->angle_max;
    scan.angle_increment = scan_->angle_increment;
    scan.time_increment = scan_->time_increment;
    scan.scan_time = scan_->scan_time;
    scan.range_min = scan_->range_min;
    scan.range_max = scan_->range_max;
    scan.ranges = scan_->ranges;
    scan.intensities = scan_->intensities;

}


// クォータニオンをオイラーに変換                                               
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

//　goalで指定した位置に近いかの判定を行う
int near_position(geometry_msgs::PoseStamped goal)
{
	double difx = robot_x - goal.pose.position.x;
	double dify = robot_y - goal.pose.position.y;
	return (sqrt(difx * difx + dify * dify) < 0.2);
}


void go_position(geometry_msgs::PoseStamped goal)

    //double k_v = 0.3; // 速度の係数の初期値
    double k_v = 3.0; // 速度の係数
    double k_w = 1.6; // 角速度の係数
	
	// 指令する速度と角速度
	double v = 0.0;
	double w = 0.0;

	//　角速度の計算
	double theta = atan2(goal.pose.position.y - robot_y, goal.pose.position.x - robot_x);
	while (theta <= -M_PI || M_PI <= theta)
	{
		if (theta <= -M_PI)
			theta = theta + 2 * M_PI;
		else
			theta = theta - 2 * M_PI;
	}

	// 現在のロボットのroll, pitch, yawを計算
	geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

	while (yaw <= -M_PI || M_PI <= yaw)
	{
		if (yaw <= -M_PI)
			yaw = yaw + 2 * M_PI;
		else
			yaw = yaw - 2 * M_PI;
	}

	theta = theta - yaw; //thetaに目標とする点に向くために必要となる角度を格納

	while (theta <= -M_PI || M_PI <= theta)
	{
		if (theta <= -M_PI)
			theta = theta + 2 * M_PI;
		else
			theta = theta - 2 * M_PI;
	}

	w = k_w * theta;

	// 速度の計算(追従する点が自分より前か後ろかで計算を変更)
	if (theta <= M_PI / 2 && theta >= -M_PI / 2)
		v = k_v * ((goal.pose.position.x - robot_x) * (goal.pose.position.x - robot_x) + (goal.pose.position.y - robot_y) * (goal.pose.position.y - robot_y));
	else
		v = -k_v * ((goal.pose.position.x - robot_x) * (goal.pose.position.x - robot_x) + (goal.pose.position.y - robot_y) * (goal.pose.position.y - robot_y));
	
	// publishする値の格納
	twist.linear.x = v;
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;
	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = w;

	// std::cout << "v: " << v << ", w: " << w << std::endl;

}
int main(int argc, char **argv)
{
	// 初期化関連
	ros::init(argc, argv, "waypoint_follower");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	// Subscriber, Publisherの定義
	ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 1000, odom_callback);
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1000);
    ros::Subscriber scan_sub = nh.subscribe("scan", 10, scanCallback);


	ros::Rate loop_rate(100);

	ros::Time start;
	ros::Time now;



	// odometryの値の初期化
	robot_x = 0.0;
	robot_y = 0.0;
	robot_r.x = 0.0;
	robot_r.y = 0.0;
	robot_r.z = 0.0;
	robot_r.w = 1.0;


	while (ros::ok())
	{
		ros::spinOnce();
		ros::Time start = ros::Time::now();


		if (flag == 0){
			goal.pose.position.x = 4.5;
    		goal.pose.position.y = 0.0;

		}

		if (flag == 1){
			goal.pose.position.x = 0.0;
    		goal.pose.position.y = 0.0;
		}
		if (flag == 2){
			goal.pose.position.x = 4.5;
    		goal.pose.position.y = 0.0;			
		}

		if (flag == 3){
			goal.pose.position.x = 0.0;
    		goal.pose.position.y = 0.0;			
		}
		if (flag == 4){
			goal.pose.position.x = 4.5;
    		goal.pose.position.y = 1.0;			
		}


		// 比例制御で近づき、近づき終えたら停止
		go_position(goal);
        if (near_position(goal))
        {
			twist.linear.x = 0.0;
			twist.angular.z = 0.0;
			flag += 1;
        }


		twist_pub.publish(twist);

		loop_rate.sleep();
	}

	return 0;
}