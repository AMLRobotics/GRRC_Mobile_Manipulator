#include <ros/ros.h>
#include <autoware_msgs/Lane.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <atomic> // std::atomic 사용

ros::Publisher is_stop_pub;
std_msgs::Bool is_stop_check;

// 전역 변수 동기화를 위해 atomic 사용
std::atomic<double> x_1(0.0), y_1(0.0), x_2(0.0), y_2(0.0), pre_vel(0.0);
std::atomic<bool> way_check(false);

const double dist_threshold = 0.8;

// 두 점 사이의 유클리드 거리를 계산하는 헬퍼 함수
double get_distance(double x1, double y1, double x2, double y2)
{
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // atomic 변수에 값 할당
    x_1 = msg->pose.position.x;
    y_1 = msg->pose.position.y;

    // 현재 로봇 위치와 최종 목적지(x_2, y_2) 간의 거리 계산
    double robot_to_point_dist = get_distance(x_1, y_1, x_2, y_2);

    // 속도가 0이고, 거리가 임계값 이내일 때 정지 신호
    if (pre_vel.load() == 0.0 && std::abs(robot_to_point_dist) <= dist_threshold)
    {
        is_stop_check.data = true;
    }

    else
    {
        is_stop_check.data = false;
    }

    is_stop_pub.publish(is_stop_check);  
}

// geometry_msgs::Twist 메시지를 구독하는 콜백
void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    pre_vel = msg->linear.x; // 세미콜론 추가
}

void waypointsCallback(const autoware_msgs::Lane::ConstPtr& msg)
{
    // 웨이포인트가 비어있지 않은지 확인
    if (!msg->waypoints.empty()) {
        way_check = true;
        // ROS 컨테이너의 크기는 size() 함수로 가져옴
        int last_index = msg->waypoints.size() - 1;
        x_2 = msg->waypoints[last_index].pose.pose.position.x;
        y_2 = msg->waypoints[last_index].pose.pose.position.y;
    }

    else {
        way_check = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detecting_mobile_base_stop");
    ros::NodeHandle nh;

    // Publisher 선언 시 타입과 토픽 이름 일치시킴. Latch는 false로 설정.
    is_stop_pub = nh.advertise<std_msgs::Bool>("/is_stop", 1); 

    ros::Subscriber sub1 = nh.subscribe("/current_pose", 1, poseCallback);
    // /twist_cmd 토픽을 사용한다면 아래 줄을 사용
    // ros::Subscriber sub2 = nh.subscribe("/twist_cmd", 10, velocityCallback); 
    ros::Subscriber sub2 = nh.subscribe("/cmd_vel", 10, velocityCallback); 
    ros::Subscriber sub3 = nh.subscribe("/final_waypoints", 1, waypointsCallback);

    ros::spin();

    return 0;
}
