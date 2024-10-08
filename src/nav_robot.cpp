#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <visualization_msgs/Marker.h>  // 追加

#include <map>
#include <math.h>
#include <vector>
#include "std_msgs/Empty.h"
#include <time.h>
#include <fstream>
#include <sstream>
#include <string>

double robot_odom_x, robot_odom_y;
double robot_x, robot_y;
double roll, pitch, yaw;

int i = 0;

geometry_msgs::Quaternion robot_r;
geometry_msgs::Twist twist;
std::map<std::string, double> params_;
geometry_msgs::PoseStamped goal;
sensor_msgs::LaserScan scan;
std::vector<geometry_msgs::PoseStamped> waypoints;
std_msgs::Int16 waypoints_A_number_now;
int waypoints_A_number_next = 0;

bool goal_flag = false;

void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;
    robot_r = msg->pose.pose.orientation;
    ROS_INFO("Current estimated pose: [robot_x: %f, robot_y: %f, theta: %f]", robot_x, robot_y, tf::getYaw(msg->pose.pose.orientation));
}

void numberCallback(const std_msgs::Int16::ConstPtr &msg)
{
    waypoints_A_number_next = msg->data;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    robot_odom_x = msg->pose.pose.position.x;
    robot_odom_y = msg->pose.pose.position.y;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_)
{
    scan = *scan_;
}

bool readWaypointsFromCSV(std::string csv_file){
    std::ifstream file(csv_file);
    if(!file){
        ROS_ERROR("Cannot open file: %s", csv_file.c_str());
        return false;
    }

    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
            std::istringstream ss(line);
            std::string token;
            geometry_msgs::PoseStamped waypoint;

            std::getline(ss, token, ',');
            waypoint.pose.position.x = std::stod(token);

            std::getline(ss, token, ',');
            waypoint.pose.position.y = std::stod(token);

            std::getline(ss, token, ',');
            waypoint.pose.position.z = std::stod(token);

            waypoint.pose.orientation.x = 1.0;
            waypoints.push_back(waypoint);
            
            ROS_INFO("Read waypoint: x=%f, y=%f, z=%f", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
        }

        file.close();
        return true;
}

void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

int near_position(geometry_msgs::PoseStamped goal)
{
    double difx = robot_x - goal.pose.position.x;
    double dify = robot_y - goal.pose.position.y;
    return (sqrt(difx * difx + dify * dify) < 0.2);
}

void go_position(geometry_msgs::PoseStamped goal)
{
    double k_v = 3.0;
    double k_w = 1.6;

    double v = 1.0;
    double w = 0.0;

    double theta = atan2(goal.pose.position.y - robot_y, goal.pose.position.x - robot_x);
    while (theta <= -M_PI || M_PI <= theta)
    {
        if (theta <= -M_PI)
            theta = theta + 2 * M_PI;
        else
            theta = theta - 2 * M_PI;
    }

    geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

    while (yaw <= -M_PI || M_PI <= yaw)
    {
        if (yaw <= -M_PI)
            yaw = yaw + 2 * M_PI;
        else
            yaw = yaw - 2 * M_PI;
    }

    theta = theta - yaw;

    while (theta <= -M_PI || M_PI <= theta)
    {
        if (theta <= -M_PI)
            theta = theta + 2 * M_PI;
        else
            theta = theta - 2 * M_PI;
    }

    w = k_w * theta;

    if (theta <= M_PI / 2 && theta >= -M_PI / 2)
        v = k_v * ((goal.pose.position.x - robot_x) * (goal.pose.position.x - robot_x) + (goal.pose.position.y - robot_y) * (goal.pose.position.y - robot_y));
    else
        v = -k_v * ((goal.pose.position.x - robot_x) * (goal.pose.position.x - robot_x) + (goal.pose.position.y - robot_y) * (goal.pose.position.y - robot_y));

    twist.linear.x = 1.0;
    twist.angular.z = w;
}

void publish_waypoints_marker(ros::Publisher &marker_pub) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "waypoints";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    

    for (auto &waypoint : waypoints) {
        geometry_msgs::Point p;
        p.x = waypoint.pose.position.x;
        p.y = waypoint.pose.position.y;
        p.z = waypoint.pose.position.z;
        marker.points.push_back(p);
    }

    marker_pub.publish(marker);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotB_move");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("ypspur_ros/odom", 1000, odom_callback);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1000);
    ros::Subscriber scan_sub = nh.subscribe("robotB/scan", 10, scanCallback);
    ros::Publisher waypoints_A_number_pub = nh.advertise<std_msgs::Int16>("waypoints_A_number_now", 10);
    ros::Subscriber waypoints_A_number_sub = nh.subscribe("waypoints_A_number_next", 10, numberCallback);
    ros::Subscriber amcl_sub = nh.subscribe("/robotB/amcl_pose", 1000, amclPoseCallback);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);  // マーカーパブリッシャー

    ros::Rate loop_rate(100);

    ros::Time start = ros::Time::now();

    robot_x = 0.0;
    robot_y = 0.0;
    robot_r.x = 0.0;
    robot_r.y = 0.0;
    robot_r.z = 0.0;
    robot_r.w = 1.0;
    
    std::string csv_file = "/home/yamaguchi-a/catkin_ws/src/waypoint_follower/csv/waypoints.csv";

    if (!readWaypointsFromCSV(csv_file)) {
        ROS_ERROR("Failed to read waypoints from CSV file.");
        return 1;
    }

    while(ros::ok()){
        ros::Time now = ros::Time::now();   
        if (now - start < ros::Duration(10.0))
        {
            continue;
        }

        ros::spinOnce();

        // waypointをRvizに表示
        publish_waypoints_marker(marker_pub);

        go_position(goal);

        if(near_position(goal)){
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            goal.pose.position.x = waypoints[i].pose.position.x;
            goal.pose.position.y = waypoints[i].pose.position.y;
            i += 1;
        }

        twist_pub.publish(twist);
        loop_rate.sleep();
    }
    return 0;
}
