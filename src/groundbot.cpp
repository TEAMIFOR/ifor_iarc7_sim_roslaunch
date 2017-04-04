#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include <string.h>
#include <time.h>
#include <stdlib.h> 

//Parameters
float v_max = 0.6;
float a_max = 0.8;
float a_max_p = 0.06;
double wait_period = 20;

//Common
bool bool_turn = false;
double angle_diff = 45;
bool calculate_turn = false; 
bool first_run = true; 
float vx_4 = v_max;
float az_4 = 0.0;
double time_begin_4 = 0;
double z_target_4 = 0;
char prev_4 = '0';
bool start_turn_4 = false;
bool quad_top_turn_4 = false;
bool is_turning_4 = false;


double time_begin_5 = 0;
double time_begin_6 = 0;
double time_begin_7 = 0;
double time_begin_8 = 0;
double time_begin_9 = 0;
double time_begin_10 = 0;
double time_begin_11= 0;
double time_begin_12 = 0;
double time_begin_13 = 0;

class Controller
{
  ros::NodeHandle n;
  ros::Publisher vel_pub_4;
  ros::Subscriber odom_sub_4;  

  ros::Subscriber contact_sub;


public:
  Controller()
  {
    vel_pub_4 = n.advertise<geometry_msgs::Twist>("/robot4/cmd_vel", 1);
    odom_sub_4 = n.subscribe("/robot4/odom", 10, &Controller::odomCb,this);
  }
  ~Controller()
  {
    
  }

  struct Quaternionm
  {
      double w, x, y, z;
  };

    void GetEulerAngles(Quaternionm q, double& yaw, double& pitch, double& roll)
  {
      const double w2 = q.w*q.w;
      const double x2 = q.x*q.x;
      const double y2 = q.y*q.y;
      const double z2 = q.z*q.z;
      const double unitLength = w2 + x2 + y2 + z2;    // Normalised == 1, otherwise correction divisor.
      const double abcd = q.w*q.x + q.y*q.z;
      const double eps = 1e-7;    // TODO: pick from your math lib instead of hardcoding.
      const double pi = 3.14159265358979323846;   // TODO: pick from your math lib instead of hardcoding.
      if (abcd > (0.5-eps)*unitLength)
      {
          yaw = 2 * atan2(q.y, q.w);
          pitch = pi;
          roll = 0;
      }
      else if (abcd < (-0.5+eps)*unitLength)
      {
          yaw = -2 * ::atan2(q.y, q.w);
          pitch = -pi;
          roll = 0;
      }
      else
      {
          const double adbc = q.w*q.z - q.x*q.y;
          const double acbd = q.w*q.y - q.x*q.z;
          yaw = ::atan2(2*adbc, 1 - 2*(z2+x2));
          pitch = ::asin(2*abcd/unitLength);
          roll = ::atan2(2*acbd, 1 - 2*(y2+x2));
      }
  }

int add_multiple_error(int times)
  {
    int error = 0;
    for (int i = 0; i<times; i++)
    {
      error += rand() % 20 - 9;
    }
    return error;
  }

   void publish_velocity(std::string identification)
  {
    if(identification == "robot4/odom")
    {
      geometry_msgs::Twist command;
      command.linear.x = vx_4;
      command.angular.z = az_4;
      command.linear.y = command.linear.z = command.angular.x = command.angular.y = 0;     
      vel_pub_4.publish(command);
    }
         
    
  }


  void odomCb(const nav_msgs::Odometry::ConstPtr& msg)
  {
    float *vx;
    double *z_target; 
    float *az;
    double *time_begin;
    bool *start_turn;
    bool *quad_top_turn;
    bool *is_turning;

    Quaternionm myq;
    double yaw = 0;
    double pitch = 0;
    double roll = 0;
    myq.x = msg->pose.pose.orientation.x;
    myq.y = msg->pose.pose.orientation.y;
    myq.z = msg->pose.pose.orientation.z;
    myq.w = msg->pose.pose.orientation.w;  
    GetEulerAngles(myq, yaw, pitch, roll);
      

    if(msg->header.frame_id == "robot4/odom")
    {
      vx = &vx_4;
      z_target = &z_target_4;
      az = &az_4;
      time_begin = &time_begin_4;
      start_turn = &start_turn_4;
      quad_top_turn = &quad_top_turn_4;
      is_turning = &is_turning_4;
    }
    

    std::string Caller = "";
      
          
    double diff = *z_target - yaw;
    diff = fabs(diff);
    double time_present = ros::Time::now().toSec();
    //ROS_INFO("time_present: %lf, time_begin: %lf, roll: %lf",time_present,time_begin,diff);
    if(diff > 0.13490 && *is_turning == true)
    {
      *time_begin = time_present;
      *vx = 0;
      *az = a_max;    
    }
    else if(diff < 0.13490 && *is_turning == true)
    {
      *is_turning = false;
      *vx = v_max;
      *az = 0;
    }

    if(!*is_turning)
    {
      *vx = v_max;
      *az = 0;
    }

    if((time_present - *time_begin) > wait_period)
    {
      angle_diff = 45 + add_multiple_error(4);    
      calculate_turn = true;
      Caller = "time"; 
    }

    if(*start_turn == true)
    {
      angle_diff = 180 + add_multiple_error(4);
      calculate_turn = true;
      *start_turn = false;
      Caller = "base";
    }

    if(*quad_top_turn == true)
    {
      angle_diff = 45 + add_multiple_error(4);
      calculate_turn = true;
      *quad_top_turn = false;
      Caller = "top";
    }


    if(calculate_turn)
    {
      *is_turning = true;    
      
      double turn_angle = (angle_diff*3.141592)/180;
      *z_target = yaw - turn_angle;
      if(*z_target > 3.14)
      {
        *z_target = 2*3.14 - *z_target;
      }
      else if(*z_target < -3.14)
      {
        *z_target = 2*3.14 + *z_target;
      }
      //ROS_INFO("target: %lf",z_target);
      *time_begin = time_present;           
      calculate_turn = false;
      *vx = -v_max*5;
    }
    publish_velocity(msg->header.frame_id);
  }
  
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "CreateController");
  Controller ic;
  time_begin_4 = time_begin_5 = time_begin_6 = time_begin_7 = time_begin_8 = time_begin_9 = time_begin_10 = time_begin_11 = time_begin_12 = time_begin_13 = ros::Time::now().toSec();
  srand (time(NULL));
  ros::spin();
return 0;
}

