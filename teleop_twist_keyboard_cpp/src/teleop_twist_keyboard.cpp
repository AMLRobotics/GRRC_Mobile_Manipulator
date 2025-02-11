#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <math.h>

#include <map>

// Map for movement keys
std::map<char, std::vector<float>> moveBindings
{
  {'i', {1, 0, 0, 0}},
  {'o', {1, 0, 0, -1}},
  {'j', {0, 0, 0, 1}},
  {'l', {0, 0, 0, -1}},
  {'u', {1, 0, 0, 1}},
  {',', {-1, 0, 0, 0}},
  {'.', {-1, 0, 0, 1}},
  {'m', {-1, 0, 0, -1}},
  {'O', {1, -1, 0, 0}},
  {'I', {1, 0, 0, 0}},
  {'J', {0, 1, 0, 0}},
  {'L', {0, -1, 0, 0}},
  {'U', {1, 1, 0, 0}},
  {'<', {-1, 0, 0, 0}},
  {'>', {-1, -1, 0, 0}},
  {'M', {-1, 1, 0, 0}},
  {'t', {0, 0, 1, 0}},
  {'b', {0, 0, -1, 0}},
  {'k', {0, 0, 0, 0}},
  {'K', {0, 0, 0, 0}}
};

// Map for speed keys
std::map<char, std::vector<float>> speedBindings
{
  {'q', {1.1, 1.1}},
  {'z', {0.9, 0.9}},
  {'w', {1.1, 1}},
  {'x', {0.9, 1}},
  {'e', {1, 1.1}},
  {'c', {1, 0.9}}
};

// Reminder message
const char* msg = R"(

Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit

)";

// Init variables
float speed(0.5); // Linear velocity (m/s)
float turn(1.0); // Angular velocity (rad/s)
float x(0), y(0), z(0), th(0); // Forward/backward/neutral direction vars
char key(' ');

// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "teleop_twist_keyboard");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // Create Twist message
  geometry_msgs::Twist twist;

  printf("%s", msg);
  printf("\rCurrent: speed %f\tturn %f | Awaiting command...\r", speed, turn);

  while(true){

    // Get the pressed key
    key = getch();

    // If the key corresponds to a key in moveBindings
    if (moveBindings.count(key) == 1)
    {
      // Grab the direction data
      x = moveBindings[key][0];
      y = moveBindings[key][1];
      z = moveBindings[key][2];
      th = moveBindings[key][3];

      printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
    }

    // Otherwise if it corresponds to a key in speedBindings
    else if (speedBindings.count(key) == 1)
    {
      // Grab the speed data
      speed = speed * speedBindings[key][0];
      turn = turn * speedBindings[key][1];

      printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
    }

    // Otherwise, set the robot to stop
    else
    {
      x = 0;
      y = 0;
      z = 0;
      th = 0;

      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03')
      {
        printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
        break;
      }

      printf("\rCurrent: speed %f\tturn %f | Invalid command! %c", speed, turn, key);
    }

    //Additional part(for acceleration)
    ros::Rate loop_time(100); 
    
    const u_int8_t acc_time = 50;
    std::vector<double> acc_sign(4);
    double acc = speed / acc_time;
    double ang_acc = turn / acc_time;
    const double threshold = acc, ang_threshold = ang_acc;
    
    while(twist.linear.x != speed * x || twist.linear.y != speed * y || twist.linear.z != speed * z || twist.angular.z != turn * th)
    {
    
      //linear x acceleration's direction
      if(twist.linear.x < x * speed) acc_sign[0] = 1;   
      else if(twist.linear.x == x * speed) acc_sign[0] = 0;
      else acc_sign[0] = -1;
      
      //linear y acceleration's direction
      if(twist.linear.y < y * speed) acc_sign[1] = 1;
      else if(twist.linear.y == y * speed) acc_sign[1] = 0;
      else acc_sign[1] = -1;
      
      //linear z acceleration's direction
      if(twist.linear.z < z * speed) acc_sign[2] = 1; 
      else if(twist.linear.z == z * speed) acc_sign[2] = 0;
      else acc_sign[2] = -1;
      
      //angular acceleration's direction
      if(twist.angular.z < th * turn) acc_sign[3] = 1;
      else if(twist.angular.z == th * turn) acc_sign[3] = 0;
      else acc_sign[3] = -1;
    
      // Update the Twist message
      twist.linear.x = twist.linear.x + acc_sign[0] * acc;
      twist.linear.y = twist.linear.y + acc_sign[1] * acc;
      twist.linear.z = twist.linear.z + acc_sign[2] * acc;

      twist.angular.x = 0;
      twist.angular.y = 0;
      twist.angular.z = twist.angular.z + acc_sign[3] * ang_acc;
      
      // Prevent to over limit speed because of slight difference of acceleration
      if(twist.linear.x > speed) twist.linear.x = speed;
      if(twist.linear.x < -speed) twist.linear.x = -speed;
      if(twist.linear.y > speed) twist.linear.y = speed;
      if(twist.linear.y < -speed) twist.linear.y = -speed;
      if(twist.linear.z > speed) twist.linear.z = speed;
      if(twist.linear.z < -speed) twist.linear.z = -speed;
      if(twist.angular.z > turn) twist.angular.z = turn;
      if(twist.angular.z < -turn) twist.angular.z = -turn;
      
      // Force to make speed to reach 0 when slight difference is occured.
      if(fabs(twist.linear.x) < threshold) twist.linear.x = 0;
      if(fabs(twist.linear.y) < threshold) twist.linear.y = 0;
      if(fabs(twist.linear.z) < threshold) twist.linear.z = 0;
      if(fabs(twist.angular.z) < ang_threshold) twist.angular.z = 0;

      // Publish it and resolve any remaining callbacks
      pub.publish(twist);
      ros::spinOnce();
      loop_time.sleep();
    }

  }

  return 0;
}
