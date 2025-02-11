#include <stdlib.h>
#include <stdio.h>
#include <ur_kinematics/ur_kin.h>
#include <math.h>
#include "ros/ros.h"
#include "ur_ros_cartesian_control/position.h"
#include "ur_ros_cartesian_control/cartesian_point.h"
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>

static struct termios initial_settings, new_settings;
 
static int peek_character = -1;

void init_keyboard()
{
    tcgetattr(0,&initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}
 
void close_keyboard()
{
    tcsetattr(0, TCSANOW, &initial_settings);
}
 
int _kbhit()
{
    unsigned char ch;
    int nread;
 
    if (peek_character != -1) return 1;
    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0,&ch,1);
    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);
    if(nread == 1)
    {
        peek_character = ch;
        return 1;
    }
    return 0;
}
 
int _getch()
{
    char ch;
 
    if(peek_character != -1)
    {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0,&ch,1);
    return ch;
}
 
int _putch(int c) {
    putchar(c);
    fflush(stdout);
    return c;
}

//Calculate inverse kinematics of UR10 to prevent from crashing between Lidar and UR.
//If results of base joint(q1) belong between Min and Max danger zone, then program send danger signal.
bool safety_check(ur_ros_cartesian_control::position::Request  &req,
         ur_ros_cartesian_control::position::Response &res)
{
    const double PI = 3.141592;
    const double MIN_danger = PI / 4, MAX_danger = 3 * PI / 4;

    double trans[4][4] = {0}, rot[3][4][4] = {0}, end[4][4] = {0};
    double roll, pitch, yaw;

    //convert degrees to radian.
    roll = req.roll * PI / 180; pitch = req.pitch * PI / 180; yaw = req.yaw * PI / 180;

    //transformation matrix setting
    trans[0][0] = trans[1][1] = trans[2][2] = trans[3][3] = 1;
    trans[0][3] = req.x; trans[1][3] = req.y; trans[2][3] = req.z;

    //rotation matrix setting
    rot[0][3][3] = rot[1][3][3] = rot[2][3][3] = rot[0][0][0] = rot[1][1][1] = rot[2][2][2] = 1;
    
    //roll(x rotation) matrix setting
    rot[0][1][1] = rot[0][2][2] = cos(roll);
    rot[0][1][2] = -sin(roll); rot[0][2][1] = sin(roll);

    //pitch(y rotation) matrix setting
    rot[1][0][0] = rot[1][2][2] = cos(pitch);
    rot[1][0][2] = sin(pitch); rot[1][2][0] = -sin(pitch);

    //yaw(z rotation) matrix setting
    rot[2][0][0] = rot[2][1][1] = cos(yaw);
    rot[2][0][1] = -sin(yaw); rot[2][1][0] = sin(yaw);

    //calcultate homogeneous transformation matrix of target location.
    for(int i = 2; i > -1; i--)
    {
        for(int j = 0; j < 4; j++)
        {
            for(int k = 0; k < 4; k++)
            {
                for(int l = 0; l < 4; l++) 
                {
                    end[j][k] += trans[j][l] * rot[i][l][k];
                }

                //printf("%f ", end[j][k]);
                //if(k == 3) printf("\n");
            }
        }

        //printf("\n");

        for(int j = 0; j < 4; j++)
        {
            for(int k = 0; k < 4; k++)
            {
                trans[j][k] = end[j][k];
                end[j][k] = 0;
            }
        }        
    }

    /*for(int i = 0; i < 16; i++)
    {
        printf("%f ", trans[i / 4][i % 4]);
        if(i % 4 == 3) printf("\n");
    }*/

    double solution[8*6]; 
    double sol_num = 0, T[16];

    //modify transformation matrix to accord with function's form.
    for(int i = 0; i < 4; i++)
    {
        T[i * 4] = trans[i][2];
        T[i * 4 + 1] = trans[i][0];
        T[i * 4 + 2] = trans[i][1];
        T[i * 4 + 3] = trans[i][3];
    }

    T[0] = -T[0]; T[3] = -T[3]; T[4] = -T[4]; T[7] = -T[7]; T[9] = -T[9]; T[10] = -T[10];

    //calculate inverse kinematics.
    sol_num = ur_kinematics::inverse(T, solution);

    printf("%f\n", sol_num);
    for(int i = 0; i < sol_num; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            printf("%f ", solution[6* i + j] / PI);
        }

        printf("\n\n");
    }
    
    //check results of base joint's degree is included between Min and Max danger zone.
    for(int i = 0; i < sol_num; i++)
    {
        if(solution[i * 6] > MIN_danger && solution[i * 6] < MAX_danger)
        {
            res.danger = true; 
            break;
        }
        else res.danger = false;
    }

   /*double T_for[16], q_for[6] = {-231.27, -87.26, -122.73, -64.13, 150.18, -359.08};

    for(int i = 0; i < 6; i++)
    {
        q_for[i] = q_for[i] * PI / 180;
    }

    ur_kinematics::forward(q_for, T_for);

    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            printf("%f ", T_for[i * 4 + j]);
        }
        printf("\n");
    }*/

    return true;
}

bool get_cartesian_position(ur_ros_cartesian_control::cartesian_point::Request  &req,
         ur_ros_cartesian_control::cartesian_point::Response &res)
{
    double T[16] = {0.0}, q[6] = {0.0};

    for(int i = 0; i < 6; i++)
    {
        q[i] = req.jointDegs[i];
    }
    
    ur_kinematics::forward(q, T);

    res.x = -T[3];
    res.y = -T[7];
    res.z = T[11];
    res.pitch = atan2(-T[1], T[0]) + 0.5 * 3.14159265;
    res.roll = -atan2(-T[6], T[10]) + 1.5 * 3.14159265;
    res.yaw = atan2(T[2], sqrt(pow(T[10], 2) + pow(T[6], 2)));

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "UR10_IK_Solver");
    ros::NodeHandle n;
    //ros::Rate loop_rate(100);
    ros::ServiceServer service_ik = n.advertiseService("motion_safety_check", safety_check);
    ros::ServiceServer service_fk = n.advertiseService("current_cartesian_position", get_cartesian_position);
    init_keyboard();

    while(ros::ok())
    {
        ros::spinOnce();

        if(_kbhit() == 1)
        {
            char ch = _getch();
            if(ch == 27) 
            {
                char ch_ext = _getch();
                if(ch_ext == 27) 
                {
                    printf("Stopping inverse kinematics solver.\n");
                    break;
                }
            }
        }
        
        //loop_rate.sleep();
    }
    //ros::spin();
    close_keyboard();
    return 0;
}