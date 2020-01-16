#include<cmath>
#include "ros/ros.h"
#include<vfly/vfly_pose.h>

//int timestamp;
ros::Publisher vfly_pose_desired_pub;
void timerCallback(const ros::TimerEvent &event)
{
    static float timestamp = 0.0f;
    //ROS_INFO("curtime : %f",(double)timestamp);
    timestamp += 0.1;
    vfly::vfly_pose pose;
    if(timestamp < 5.f )
    {
        pose.roll = 0.f;
        
    }
    else if(5.0f <= timestamp &&timestamp < 13.0f)
    {
        pose.roll  = -180.f*sinf(M_PI/16.f*(timestamp-5.f));
    }
   else if(13.0f <= timestamp &&timestamp < 21.0f)
   {
        pose.roll= 180.f*cosf(M_PI/16.f*(timestamp-13.f));
   }
   else 
   {
       timestamp = 5.f;
   }
    pose.x = 0.0f;
    pose.y = 0.f;
    pose.z = 3.f;
    pose.pitch = 0.0f;
    pose.yaw = 0.0f;
    pose.header.stamp = ros::Time::now();
    vfly_pose_desired_pub.publish(pose);
}
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rotation_on_fixed_spot");
  ros::NodeHandle nh;
  ros::Timer _control_timer = nh.createTimer(ros::Duration(0.1), &timerCallback);
  ros::NodeHandle private_nh("~");
  vfly_pose_desired_pub = nh.advertise<vfly::vfly_pose>("command/pose",1);
  ros::spin();
  return 0;
}

