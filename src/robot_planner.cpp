#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <sstream>
#include <vector>

class RobotPlanner
{
public:
    RobotPlanner();  // コンストラクタ
    bool readWaypointsFromCSV(const std::string& csv_file);  // CSVからウェイポイントを読み込む関数
    void publishPlan();  // プランのパブリッシュ関数

private:
    ros::NodeHandle nh_;                  // ノードハンドル
    ros::Publisher waypoint_pub_;         // ウェイポイントパブリッシャー
    ros::Publisher max_speed_pub_;            // 速度指令パブリッシャー
    std::vector<geometry_msgs::PoseStamped> waypoints;  // ウェイポイントを格納するベクタ
    int current_waypoint_index_;          // 現在のウェイポイントインデックス
};

// コンストラクタ
RobotPlanner::RobotPlanner() 
    : current_waypoint_index_(0)
{
    // ウェイポイントのパブリッシュ
    waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("waypoint", 1000);
    // 速度指令のパブリッシュ
    max_speed_pub_ = nh_.advertise<std_msgs::Float32>("max_speed", 1000);//namespace確認
}

// CSVファイルからウェイポイントを読み込む関数
bool RobotPlanner::readWaypointsFromCSV(const std::string& csv_file)
{
    std::ifstream file(csv_file);  // csvファイルを開く
    if (!file) {  // ファイルが開けなかった場合
        ROS_ERROR("Cannot open file: %s", csv_file.c_str());
        return false;
    }

    std::string line;
    std::getline(file, line);  // 最初の一行（ヘッダー行）を読み飛ばす

    while (std::getline(file, line)) {
        std::istringstream ss(line);  // 文字列をカンマで区切るための変数
        std::string token;
        geometry_msgs::PoseStamped waypoint;  // ウェイポイントを格納する変数
        
        // x座標の読み込み
        std::getline(ss, token, ',');
        waypoint.pose.position.x = std::stod(token);  // 文字列をdouble型に変換してwaypointに格納

        // y座標の読み込み
        std::getline(ss, token, ',');
        waypoint.pose.position.y = std::stod(token);

        // z座標の読み込み
        std::getline(ss, token, ',');
        waypoint.pose.position.z = std::stod(token);

        // orientationを初期化
        waypoint.pose.orientation.x = 1.0;

        // ベクトルにウェイポイントを追加
        waypoints.push_back(waypoint);

        // デバッグメッセージを出力
        ROS_INFO("Read waypoint: x=%f, y=%f, z=%f", waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z);
    }

    file.close();
    return true;
}

// ウェイポイントのパブリッシュ
void RobotPlanner::publishPlan()
{
    if (current_waypoint_index_ < waypoints.size()) {
        geometry_msgs::PoseStamped current_waypoint = waypoints[current_waypoint_index_];

        // ウェイポイントをパブリッシュ
        waypoint_pub_.publish(current_waypoint);

        // ウェイポイント情報をログに表示
        ROS_INFO("Publishing waypoint: x=%f, y=%f, z=%f",
                 current_waypoint.pose.position.x,
                 current_waypoint.pose.position.y,
                 current_waypoint.pose.position.z);

        // 速度指令を作成してパブリッシュ (仮の指令値として、一定速度を与える)
        std_msgs::Float32 max_speed; 
        max_speed.data = 1.0;  

        max_speed_pub_.publish(max_speed);

        // ウェイポイントを次に進める
        current_waypoint_index_++;
    } else {
        ROS_WARN("No more waypoints to publish.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_planner_node");

    RobotPlanner planner;

    std::string csv_file = "/home/yamaguchi-a/catkin_ws/src/waypoint_follower/csv/waypoints_A.csv";    

    if (!planner.readWaypointsFromCSV(csv_file)) {
        return 1;  // エラーが発生した場合、ノードを終了
    }

    ros::Rate loop_rate(10);  // 10Hzのループレート

    while (ros::ok())
    {
        // プランのパブリッシュ
        planner.publishPlan();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
