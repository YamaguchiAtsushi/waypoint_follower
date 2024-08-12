#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>


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
std_msgs::Int16 waypoints_A_number_now; // 今向かっているロボットAのwaypointの番号;
//std_msgs::Int16 waypoints_A_number_next; // 送られてくる次に向かうロボットAのwaypointの番号
geometry_msgs::PoseWithCovarianceStamped current_pose;

//int waypoints_A_number_now = 0; //今向かっているロボットAのwaypointの番号
//int waypoints_A_number_next = 0; // 送られてくる次に向かうロボットAのwaypointの番号
int waypoints_A_number_next = 0; // 送られてくる次に向かうロボットAのwaypointの番号
//int waypoints_A_number_next = 1; // 送られてくる次に向かうロボットAのwaypointの番号

bool goal_flag = false;

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    current_pose = *msg;
    robot_x = current_pose.pose.pose.position.x;
    robot_y = current_pose.pose.pose.position.y;
    robot_r = current_pose.pose.pose.orientation;
    ROS_INFO("Current estimated pose: [robot_x: %f, robot_y: %f, theta: %f]", robot_x, robot_y, tf::getYaw(current_pose.pose.pose.orientation));
}

void numberCallback(const std_msgs::Int16::ConstPtr &msg)
{
    waypoints_A_number_next = msg->data;


}

// オドメトリのコールバック
// void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
// {
// 	robot_x = msg->pose.pose.position.x;
// 	robot_y = msg->pose.pose.position.y;
// 	robot_r = msg->pose.pose.orientation;
// }

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

bool readWaypointsFromCSV(std::string csv_file){
    std::ifstream file(csv_file); //csvファイルを開く
    if(!file){ //ファイルが開けなかった場合
        ROS_ERROR("Cannot open file: %s", csv_file.c_str());
        return false;
    }

    std::string line; //1行ずつ読み込むための変数

    std::getline(file, line); //最初の一行を読み飛ばす

    while (std::getline(file, line)) {
            std::istringstream ss(line);//文字列をカンマで区切るための変数
            std::string token;//1つのデータを格納する変数
            geometry_msgs::PoseStamped waypoint; //waypointを格納する変数

            // x座標の読み込み
            std::getline(ss, token, ',');//カンマで区切った文字列を1つずつ読み込む
            waypoint.pose.position.x = std::stod(token);//文字列をdouble型に変換してwaypointに格納

            // y座標の読み込み
            std::getline(ss, token, ',');
            waypoint.pose.position.y = std::stod(token);

            // z座標の読み込み
            std::getline(ss, token, ',');
            waypoint.pose.position.z = std::stod(token);

            // orientationを初期化
            waypoint.pose.orientation.x = 1.0;

            // ベクトルに追加
            waypoints.push_back(waypoint);
            
            // デバッグメッセージを追加
            ROS_INFO("Read waypoint: x=%f, y=%f, z=%f", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
        }

        file.close();
        return true;
    }



// クォータニオンをオイラーに変換                                               
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

// void rpy_to_geometry_quat(double roll, double pitch, double yaw, geometry_msgs::Quaternion &geometry_quat)
// {
//     tf::Quaternion quat_;
//     quat_.setRPY(roll, pitch, yaw);
//     quaternionTFToMsg(quat_, geometry_quat);
// }

//　goalで指定した位置に近いかの判定を行う
int near_position(geometry_msgs::PoseStamped goal)
{
	double difx = robot_x - goal.pose.position.x;
	double dify = robot_y - goal.pose.position.y;
	return (sqrt(difx * difx + dify * dify) < 0.2);
}

void go_position(geometry_msgs::PoseStamped goal)
{
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
	//twist.linear.x = v;//defalult
	twist.linear.x = 1.0;

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
	ros::init(argc, argv, "robotA_move");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");

	// Subscriber, Publisherの定義
	// ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 1000, odom_callback);
    // ros::Publisher initial_pose = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1);
	ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1000);
    ros::Subscriber scan_sub = nh.subscribe("robotA/scan", 10, scanCallback);
    ros::Publisher waypoints_A_number_pub = nh.advertise<std_msgs::Int16>("waypoints_A_number_now", 10);
    ros::Subscriber waypoints_A_number_sub = nh.subscribe("waypoints_A_number_next", 10, numberCallback);
    ros::Subscriber amcl_sub = nh.subscribe("robotA/amcl_pose", 1000, amclPoseCallback);

	ros::Rate loop_rate(100);

    ros::Time start = ros::Time::now();

	// odometryの値の初期化
	robot_x = 0.0;
	robot_y = 0.0;
	robot_r.x = 0.0;
	robot_r.y = 0.0;
	robot_r.z = 0.0;
	robot_r.w = 1.0;
    
    std::string csv_file = "/home/yamaguchi-a/catkin_ws/src/waypoint_follower/csv/waypoints_A.csv";

    

    if (!readWaypointsFromCSV(csv_file)) {
        ROS_ERROR("Failed to read waypoints from CSV file.");
        return 1;
    }
    


    while(ros::ok()){

        ros::Time now = ros::Time::now();   
        if (now - start < ros::Duration(3.0))
        {
            continue;
        }

        ros::spinOnce();

        waypoints_A_number_now.data = waypoints_A_number_next;
        goal.pose.position.x = waypoints[waypoints_A_number_next].pose.position.x;
        goal.pose.position.y = waypoints[waypoints_A_number_next].pose.position.y;

        std::cout << "waypoints_A_number_now.data:" << waypoints_A_number_now.data << std::endl;
        std::cout << "waypoints_A_number_next" << waypoints_A_number_next << std::endl;
        std::cout << goal.pose.position.x << " " << goal.pose.position.y << std::endl;

        go_position(goal);
        std::cout << "goal_flag:" << goal_flag << std::endl;


        if(near_position(goal)){

            std::cout << "goal_flag:" << goal_flag << std::endl;
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;

            waypoints_A_number_pub.publish(waypoints_A_number_now);

            }


        



        twist_pub.publish(twist);
        //waypoints_A_number_pub.publish(waypoints_A_number_now);
        loop_rate.sleep();
    }
	return 0;
}