#include <ros/ros.h>
#include <ros/topic.h>
#include <std_msgs/String.h> // 실제 사용하는 메시지 타입으로 변경 필요
#include <std_msgs/Bool.h>
#include <cstdlib>
#include <string>
#include <vector>

// 데모의 각 스텝을 정의하는 구조체
struct DemoStep {
    std::string demo_set;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "senario_controller");
    ros::NodeHandle nh;

    // 매니퓰레이터 제어 명령 퍼블리셔
    ros::Publisher mani_pub = nh.advertise<std_msgs::String>("/manipulator_task_command", 1);

    // 전체 데모 시나리오 구성
    std::vector<DemoStep> demo_sequence = {
        {"UR PICK_OBJECT_1"}, {"AW /autoware/demo_waypoint_1.csv"},
        {"UR PICK_OBJECT_2"}, {"UR PLACE_OBJECT_1"},
        {"AW /autoware/demo_waypoint2.csv"}, {"UR PLACE_OBJECT_2"}
        // 필요한 만큼 스텝 추가
    };

    ROS_INFO("===== Mobile Manipulator Demo Sequence =====");
    ROS_INFO_STREAM("Press ENTER to Start Demo.");
    std::cin.get(); // 영상 촬영을 위한 반자동 제어 (원치 않으면 주석 처리)

    for (size_t i = 0; i < demo_sequence.size(); ++i) 
    {
        std::string subject = demo_sequence[i].demo_set.substr(0, demo_sequence[i].demo_set.find(' '));
        std::string command = demo_sequence[i].demo_set.substr(demo_sequence[i].demo_set.find(' ') + 1);

        if(subject == "UR")
        {
            std_msgs::String cmd_msg;
            cmd_msg.data = command;
            mani_pub.publish(cmd_msg);

            // 2. Waiting Manipulation Task
            ROS_INFO("Waiting for Manipulation Task...");
            boost::shared_ptr<std_msgs::String const> mani_done_msg;
            mani_done_msg = ros::topic::waitForMessage<std_msgs::String>("/manipulator_task_end", nh);
        }

        else if (subject == "AW") 
        {
            // [자율주행 제어 분기]
            // launch 명령어에 잘라낸 command(경로)를 합침
            std::string launch_cmd = "roslaunch waypoint_maker waypoint_loader.launch filename:=" + command + " &";
            ROS_INFO_STREAM("[" << i + 1 << "] Moving to Destination: " << command);
            int ret = system(launch_cmd.c_str());

            // 주행 완료 대기 (True 값 검증 로직 추가)
            ROS_INFO("Waiting for Moving Task to finish...");
            while (ros::ok()) {
                auto arrival_msg = ros::topic::waitForMessage<std_msgs::Bool>("/is_stop", nh);
                if (arrival_msg && arrival_msg->data == true) {
                    ROS_INFO("Arrived at Destination.");
                    break;
                }
            }

            // Autoware 주행 노드 강제 종료 (스톱)
            system("pkill -f waypoint_loader");
        }

        ros::Duration(2.0).sleep(); // 로봇이 완전히 멈추고 진동이 잦아들 시간 부여
    }

    ROS_INFO("All Demo Senario is Done.");
    return 0;
}