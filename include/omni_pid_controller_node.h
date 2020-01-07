#ifndef OMNI_ADRC_CONTROLLER_NODE_H
#define OMNI_ADRC_CONTROLLER_NODE_H

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdlib>
#include <math.h>
#include <mav_msgs/Actuators.h>
#include <std_msgs/Float32.h>
#include <sstream>
#include <string>
#include "pid.h"
#include "vfly/vfly_pose.h"

class OmniPidControllerNode
{
public:
  OmniPidControllerNode(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);
  ~OmniPidControllerNode();

  void InitializeParams();
  void OdometryCallback(nav_msgs::Odometry odometry);
  void TimedControlCallback(const ros::TimerEvent &event);
  void EulerBasedControl(double dt);
  void QuaternionBasedControl(double dt);
  void ControlAllocation();
  void CommandPosCallback(vfly::vfly_pose pose);
  void Constraint(float &val, float limit);
  void QtoEuler(Eigen::Vector3d &rpy, const Eigen::Quaterniond &Q);
  void EulertoQ(const Eigen::Vector3d &rpy, Eigen::Quaterniond &Q);
  double Sgn(double val);
  void GetRosParameter(const ros::NodeHandle &nh, const std::string &key, const float &default_value, float *value);
  Eigen::MatrixXd Pinv(Eigen::MatrixXd A);

private:
  ros::NodeHandle _nh;
  ros::NodeHandle _private_nh;

  Eigen::Vector3d _position_gain;  //位置外环比例增益
  Eigen::Vector3d _attitude_gain;   //姿态外环比例增益


  PID_t _pid_u, _pid_v, _pid_w; //速度PID控制器
  PID_t _pid_p, _pid_q, _pid_r; //角速度PID控制器

  double _sensors_lastsecs;           //上一次控制时间戳
  double _control_lastsecs;
  float _mass, _Ix, _Iy, _Iz; //飞行器质量、惯量

  Eigen::Quaterniond _state_q; //飞行器状态数据：姿态四元素
  Eigen::Vector3d _state_rpy;
  Eigen::Vector3d _state_pos;  //飞行器状态数据：位置

  Eigen::Vector3d _state_omega; //飞行器状态数据：机体角速度
  Eigen::Vector3d _state_rates; //飞行器状态数据：全局速度

  Eigen::Vector3d _command_pos;  //位置期望
  Eigen::Vector3d _command_rpy;  //姿态期望
  Eigen::Quaterniond _command_q; //姿态期望

  Eigen::Vector3d _F_e; //位置控制器输出：全局控制力期望
  Eigen::Vector3d _M_b; //姿态控制器输出：机体控制力距期望

  Eigen::Matrix<double, 8, 6> _allocationMatrix; //控制分配矩阵：机体系下控制力;控制力距 to 旋翼转速和倾转角度

  ros::Publisher _actuator_servo_position_pub; //发布舵机角度期望（旋翼倾转角度）
  ros::Publisher _actuator_motor_speed_pub;    //发布旋翼转速期望
  
  ros::Publisher _state_r_pub;
  ros::Publisher _state_p_pub;
  ros::Publisher _state_y_pub;

  ros::Subscriber _sensor_odemetry_sub;  //订阅里程计数据：位置 姿态q 速度 角速度
  ros::Subscriber _command_pose_sub;     //订阅外部控制指令：位置期望、姿态期望rpy

  ros::Timer _control_timer;
  bool _odometry_got;
};
#endif

