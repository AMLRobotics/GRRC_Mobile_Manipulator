#include <ros/ros.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/Lane.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <cmath>
#include <limits>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>
#include <thread>
#include <atomic>

// Global publisher object
ros::Publisher base_waypoints_pub;
ros::Publisher cmd_vel_pub;
ros::Publisher closest_waypoint_pub;

autoware_msgs::Lane first_lane;
autoware_msgs::Lane current_waypoints;
geometry_msgs::PoseStamped current_pose;

// 토픽 발행 상태를 관리하는 전역 플래그
std::atomic<bool> publish_active(false);
geometry_msgs::Twist vel;

void laneArrayCallback(const autoware_msgs::LaneArray::ConstPtr& msg)
{
    // Check if the received LaneArray is not empty
    if (!msg->lanes.empty())
    {
        // Extract the first lane message from the array
        first_lane = msg->lanes[0];
        current_waypoints = first_lane;


        // Publish the single Lane message to the /base_waypoints topic
        base_waypoints_pub.publish(first_lane);

        // Optional: Log success message
        // ROS_INFO("Published a single Lane message with %zu waypoints.", first_lane.waypoints.size());
    }
    // else
    // {
    //     ROS_WARN("Received empty LaneArray message. Nothing published.");
    // }
}

void VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    // Extract the first lane message from the array
    vel = msg->twist;

    // Publish the single Lane message to the /base_waypoints topic
    if (publish_active.load())
    {
        cmd_vel_pub.publish(vel);
    }
}

// 두 점 사이의 유클리드 거리를 계산하는 헬퍼 함수
double get_distance(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

// 현재 위치 콜백
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
    
    if (current_waypoints.waypoints.empty()) {
        return;
    }

    int closest_index = -1;
    double min_distance = std::numeric_limits<double>::max();

    // 현재 경로의 모든 웨이포인트를 반복하며 가장 가까운 것 찾기
    for (size_t i = 0; i < current_waypoints.waypoints.size(); ++i) {
        double wx = current_waypoints.waypoints[i].pose.pose.position.x;
        double wy = current_waypoints.waypoints[i].pose.pose.position.y;
        double px = current_pose.pose.position.x;
        double py = current_pose.pose.position.y;

        double distance = get_distance(px, py, wx, wy);

        if (distance < min_distance) {
            min_distance = distance;
            closest_index = i;
        }
    }

    // 가장 가까운 웨이포인트 인덱스 발행
    if (closest_index != -1) {
        std_msgs::Int32 index_msg;
        index_msg.data = closest_index;
        closest_waypoint_pub.publish(index_msg);
    }
}

// 터미널에서 엔터 없이 키 입력을 받기 위한 함수
int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt); // 현재 터미널 설정 저장
    newt = oldt;
    newt.c_lflag &= ~(ICANON);      // 캐노니컬 모드(엔터 필요) 비활성화
    tcsetattr( STDIN_FILENO, TCSANOW, &newt); // 새 설정 적용
    int c = getchar();              // 문자 읽기
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt); // 이전 설정 복원
    return c;
}

// 키 입력을 전담하는 별도 스레드 함수
void keyboard_input_thread(ros::Publisher& pub)
{
    while (ros::ok())
    {
        int c = getch();

        if (c == 's') {
            // 's' 키를 누르면 발행 활성화 및 메시지 설정
            publish_active.store(true);
            ROS_INFO("TOPIC ADIVERTISE STARTED");
        } else if (c == 'p') {
            // 'p' 키를 누르면 발행 비활성화 (Pause)
            publish_active.store(false);
            ROS_INFO("TOPIC ADIVERTISE PAUSED)");
        } else if (c == 'q') {
            // 'q' 키를 누르면 발행 비활성화 및 종료 준비
            publish_active.store(false);
            ROS_INFO("QUIT COMMAND REQUESTED.");
            ros::shutdown(); // ROS 시스템 종료 요청
            break;
        }
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "lane_array_to_lane_converter_cpp");
    ros::NodeHandle nh;

    // Advertise the /base_waypoints topic. 
    // The queue size is set to 1, as we only need the latest lane.
    base_waypoints_pub = nh.advertise<autoware_msgs::Lane>("/base_waypoints", 1, true);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10, true);
    closest_waypoint_pub = nh.advertise<std_msgs::Int32>("/closest_waypoint", 1);

    // Subscribe to the topic published by astar_navi (or op_global_planner)
    ros::Subscriber sub1 = nh.subscribe("/lane_waypoints_array", 1, laneArrayCallback);
    ros::Subscriber sub2 = nh.subscribe("/twist_cmd", 1, VelocityCallback);
    ros::Subscriber pose_sub = nh.subscribe("/current_pose", 1, poseCallback);

    std::thread input_thread(keyboard_input_thread, std::ref(cmd_vel_pub));

    ROS_INFO("C++ LaneArray to Lane converter node started and ready.");
    ROS_INFO("------------------------------------------");
    ROS_INFO("  Press 's' to START continuous publishing ('START_CONTINUOUS')");
    ROS_INFO("  Press 'p' to PAUSE publishing");
    ROS_INFO("  Press 'q' to QUIT");
    ROS_INFO("------------------------------------------");

    /*...TODO...*/

    ros::spin();

    // 메인 루프 종료 후 스레드 정리
    if (input_thread.joinable()) 
    {
        input_thread.join();
    } 

    return 0;
}


