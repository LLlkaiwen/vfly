#include "omni_adrc_controller_node.h"

OmniAdrcControllerNode::OmniAdrcControllerNode(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh)
    : _nh(nh)
    , _private_nh(private_nh)
    , _attitude_gain(3.0, 3.0, 3.0)
    , _state_q(1.0, 0.0, 0.0, 0.0)
    , _state_pos(0.0, 0.0, 0.0)
    , _state_rpy(0.0, 0.0, 0.0)
    , _state_omega(0.0, 0.0, 0.0)
    , _state_rates(0.0, 0.0, 0.0)
    , _command_rpy(0.0, 0.0, 0.0)
    , _command_q(1.0,0.0,0.0,0.0)
    , _command_pos(0.0, 0.0, 3.0)
    , _F_e(0.0, 0.0, 0.0)
    , _M_b(0.0, 0.0, 0.0)
    , _odometry_got(false)
{
  InitializeParams();

  _actuator_servo_position_pub = _nh.advertise<mav_msgs::Actuators>("gazebo/command/servo_position", 1);

  _actuator_motor_speed_pub = _nh.advertise<mav_msgs::Actuators>("gazebo/command/motor_speed", 1);

  _sensor_odemetry_sub = _nh.subscribe("ground_truth/odometry", 1, &OmniAdrcControllerNode::OdometryCallback, this);

  _command_pose_sub = _nh.subscribe("command/pose", 1, &OmniAdrcControllerNode::CommandPoseCallback, this);

  _state_r_pub = _nh.advertise<std_msgs::Float64>("state/roll", 1);
  _state_p_pub = _nh.advertise<std_msgs::Float64>("state/pitch", 1);
  _state_y_pub = _nh.advertise<std_msgs::Float64>("state/yaw", 1);
  _roll_eso_z2_pub = _nh.advertise<std_msgs::Float64>("roll_eso_z2", 1);
  _roll_eso_z1_pub = _nh.advertise<std_msgs::Float64>("roll_eso_z1", 1);

  _control_timer = _nh.createTimer(ros::Duration(0.004), &OmniAdrcControllerNode::TimedControlCallback, this);
  _control_lastsecs  = _sensors_lastsecs = ros::Time::now().toSec();
}

OmniAdrcControllerNode::~OmniAdrcControllerNode()
{
}

void OmniAdrcControllerNode::InitializeParams()
{
  double kf, km, l;
  ////从 rosparam 读取飞行器参数 初始化控制分配矩阵

  GetRosParameter(_private_nh, "vehicle/kf", 1.7088e-05, &kf);
  GetRosParameter(_private_nh, "vehicle/km", 1.7088e-05 * 0.016, &km);
  GetRosParameter(_private_nh, "vehicle/l", 0.355, &l);
  GetRosParameter(_private_nh, "vehicle/mass", 2.274, &_mass);
  GetRosParameter(_private_nh, "vehicle/Ix", 0.021968, &_Ix);
  GetRosParameter(_private_nh, "vehicle/Iy", 0.021968, &_Iy);
  GetRosParameter(_private_nh, "vehicle/Iz", 0.042117, &_Iz);

    std::string type = std::string("+");
  _private_nh.getParam("vehicle/type",type);
  
  Eigen::Matrix<double, 6, 8> angular_theta_to_fm_b;
  if(type == "x")
  {
  angular_theta_to_fm_b <<  -kf*sqrtf(2.0f)/2.0f,   0,  -kf*sqrtf(2.0f)/2.0f,  0,  kf*sqrtf(2.0f)/2.0f,  0,   kf*sqrtf(2.0f)/2.0f,  0,  
                                                            -kf*sqrtf(2.0f)/2.0f,  0,    kf*sqrtf(2.0f)/2.0f,  0,  kf*sqrtf(2.0f)/2.0f,  0,  -kf*sqrtf(2.0f)/2.0f,  0, 
                                                             0,  kf, 0, kf, 0, kf, 0, kf, 
                                                            km*sqrtf(2.0f)/2.0f, -kf*sqrtf(2.0f)/2.0f*l, -km*sqrtf(2.0f)/2.0f, -kf*sqrtf(2.0f)/2.0f*l, -km*sqrtf(2.0f)/2.0f, kf*sqrtf(2.0f)/2.0f*l ,km*sqrtf(2.0f)/2.0f, kf*sqrtf(2.0f)/2.0f*l,
                                                            km*sqrtf(2.0f)/2.0f, -kf*sqrtf(2.0f)/2.0f*l,  km*sqrtf(2.0f)/2.0f,  kf*sqrtf(2.0f)/2.0f*l, -km*sqrtf(2.0f)/2.0f, kf*sqrtf(2.0f)/2.0f*l ,-km*sqrtf(2.0f)/2.0f, -kf*sqrtf(2.0f)/2.0f*l,
                                                          -l * kf, -km, -l * kf, km, -l * kf, -km, -l * kf,  km;
  }
  else 
  {
  angular_theta_to_fm_b << 0,  0, -kf, 0, 0, 0, kf, 0, 
                                                        -kf, 0, 0, 0, kf, 0, 0, 0,
                                                         0, kf, 0, kf, 0, kf, 0, kf, 
                                                         0, 0, -km, -l * kf, 0, 0, -km, l * kf,
                                                          -km, -l * kf, 0, 0, -km, l * kf, 0, 0, 
                                                          -l * kf, km, -l * kf, km, -l * kf, -km, -l * kf, -km;
  }

  // Eigen::Matrix<double, 6, 8> angular_theta_to_fm_b;
  // angular_theta_to_fm_b << 0, 0, -kf, 0, 0, 0, kf, 0, -kf, 0, 0, 0, kf, 0, 0, 0, 0, kf, 0, kf, 0, kf, 0, kf, 0, 0, km,
  //     -l * kf, 0, 0, -km, l * kf, -km, -l * kf, 0, 0, -km, l * kf, 0, 0, -l * kf, km, -l * kf, km, -l * kf, -km,
  //     -l * kf, -km;
  _allocationMatrix = Pinv(angular_theta_to_fm_b);
  std::cout << "FM_b_to_actuator allocation:\n"
            << _allocationMatrix << std::endl;

    //从 rosparam 读取控制器参数 初始化控制器

  // adrc_cascade 位置串级控制器参数
  {
    double limit;
    GetRosParameter(_private_nh, "adrc_cascade/adrc_x/td_r", 2, &_adrc_x._para_td_r);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_x/h", 0.005, &_adrc_x._para_h);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_x/b0", 0.5, &_adrc_x._para_b0);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_x/eso_beta1", 200, &_adrc_x._para_eso_beta1);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_x/eso_beta2", 20000, &_adrc_x._para_eso_beta2);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_x/outer_nlsef_k", 1, &_adrc_x._para_outer_nlsef_k);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_x/outer_nlsef_alpha", 0.5, &_adrc_x._para_outer_nlsef_alpha);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_x/outer_nlsef_delta", 0.5, &_adrc_x._para_outer_nlsef_delta);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_x/inner_nlsef_k", 10, &_adrc_x._para_inner_nlsef_k);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_x/outer_nlsef_alpha", 0.5, &_adrc_x._para_outer_nlsef_alpha);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_x/outer_nlsef_delta", 0.5, &_adrc_x._para_outer_nlsef_delta);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_x/limit", 18, &limit);
    _adrc_x.SetLimit(limit, -limit);
    std::cout << "adrc_x 参数" << std::endl;
    _adrc_x.PrintParameters(std::cout);

    GetRosParameter(_private_nh, "adrc_cascade/adrc_y/td_r", 2, &_adrc_y._para_td_r);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_y/h", 0.005, &_adrc_y._para_h);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_y/b0", 0.5, &_adrc_y._para_b0);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_y/eso_beta1", 200, &_adrc_y._para_eso_beta1);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_y/eso_beta2", 20000, &_adrc_y._para_eso_beta2);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_y/outer_nlsef_k", 1, &_adrc_y._para_outer_nlsef_k);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_y/outer_nlsef_alpha", 0.5, &_adrc_y._para_outer_nlsef_alpha);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_y/outer_nlsef_delta", 0.5, &_adrc_y._para_outer_nlsef_delta);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_y/inner_nlsef_k", 10, &_adrc_y._para_inner_nlsef_k);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_y/outer_nlsef_alpha", 0.5, &_adrc_y._para_outer_nlsef_alpha);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_y/outer_nlsef_delta", 0.5, &_adrc_y._para_outer_nlsef_delta);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_y/limit", 18, &limit);
    _adrc_y.SetLimit(limit, -limit);
    std::cout << "adrc_y 参数" << std::endl;
    _adrc_y.PrintParameters(std::cout);

    GetRosParameter(_private_nh, "adrc_cascade/adrc_z/td_r", 2, &_adrc_z._para_td_r);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_z/h", 0.005, &_adrc_z._para_h);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_z/b0", 0.5, &_adrc_z._para_b0);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_z/eso_beta1", 200, &_adrc_z._para_eso_beta1);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_z/eso_beta2", 20000, &_adrc_z._para_eso_beta2);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_z/outer_nlsef_k", 1, &_adrc_z._para_outer_nlsef_k);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_z/outer_nlsef_alpha", 0.5, &_adrc_z._para_outer_nlsef_alpha);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_z/outer_nlsef_delta", 0.5, &_adrc_z._para_outer_nlsef_delta);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_z/inner_nlsef_k", 10, &_adrc_z._para_inner_nlsef_k);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_z/outer_nlsef_alpha", 0.5, &_adrc_z._para_outer_nlsef_alpha);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_z/outer_nlsef_delta", 0.5, &_adrc_z._para_outer_nlsef_delta);
    GetRosParameter(_private_nh, "adrc_cascade/adrc_z/limit", 18, &limit);
    _adrc_z.SetLimit(limit, -limit);
    std::cout << "adrc_z 参数" << std::endl;
    _adrc_z.PrintParameters(std::cout);
  }

  {
    double gain;
    GetRosParameter(_private_nh, "attitude_gain/x", 3, &gain);
    _attitude_gain(0) = gain;
    GetRosParameter(_private_nh, "attitude_gain/y", 3, &gain);
    _attitude_gain(1) = gain;
    GetRosParameter(_private_nh, "attitude_gain/z", 3, &gain);
    _attitude_gain(2) = gain;
  }

  //adrc_firstorder 角速度环控制器
  {
    double limit;
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_roll/td_r", 2, &_adrc_roll._para_td_r);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_roll/h", 0.005, &_adrc_roll._para_h);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_roll/b0", 21, &_adrc_roll._para_b0);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_roll/leso_beta1", 200, &_adrc_roll._para_leso_beta1);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_roll/leso_beta2", 20000, &_adrc_roll._para_leso_beta2);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_roll/lsef_k1", 10, &_adrc_roll._para_lsef_k1);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_roll/lsef_k2", 10, &_adrc_roll._para_lsef_k2);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_roll/limit", 3, &limit);
    _adrc_roll.SetLimit(limit, -limit);
    std::cout << std::endl
              << "adrc_rollspeed 参数" << std::endl;
    _adrc_roll.PrintParameters(std::cout);

    GetRosParameter(_private_nh, "adrc_firstorder/adrc_pitch/td_r", 2, &_adrc_pitch._para_td_r);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_pitch/h", 0.005, &_adrc_pitch._para_h);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_pitch/b0", 21, &_adrc_pitch._para_b0);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_pitch/leso_beta1", 200, &_adrc_pitch._para_leso_beta1);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_pitch/leso_beta2", 20000, &_adrc_pitch._para_leso_beta2);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_pitch/lsef_k1", 10, &_adrc_pitch._para_lsef_k1);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_pitch/lsef_k2", 10, &_adrc_pitch._para_lsef_k2);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_pitch/limit", 3, &limit);
    _adrc_pitch.SetLimit(limit, -limit);
    std::cout << std::endl
              << "adrc_pitchspeed 参数" << std::endl;
    _adrc_pitch.PrintParameters(std::cout);

    GetRosParameter(_private_nh, "adrc_firstorder/adrc_yaw/td_r", 2, &_adrc_yaw._para_td_r);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_yaw/h", 0.005, &_adrc_yaw._para_h);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_yaw/b0", 10, &_adrc_yaw._para_b0);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_yaw/leso_beta1", 200, &_adrc_yaw._para_leso_beta1);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_yaw/leso_beta2", 20000, &_adrc_yaw._para_leso_beta2);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_yaw/lsef_k1", 10, &_adrc_yaw._para_lsef_k1);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_yaw/lsef_k2", 10, &_adrc_yaw._para_lsef_k2);
    GetRosParameter(_private_nh, "adrc_firstorder/adrc_yaw/limit", 3, &limit);
    _adrc_yaw.SetLimit(limit, -limit);
    std::cout << std::endl
              << "adrc_yawspeed 参数" << std::endl;
    _adrc_yaw.PrintParameters(std::cout);
  }
}

void OmniAdrcControllerNode::QtoEuler(Eigen::Vector3d &rpy, const Eigen::Quaterniond &Q)
{
  rpy(0) = atan2(2 * (Q.w() * Q.x() + Q.y() * Q.z()), 1 - 2 * (Q.x() * Q.x() + Q.y() * Q.y()));
  rpy(1) = asin(2 * (Q.w() * Q.y() - Q.x() * Q.z()));
  rpy(2) = atan2(2 * (Q.w() * Q.z() + Q.y() * Q.x()), 1 - 2 * (Q.z() * Q.z() + Q.y() * Q.y()));
}

void OmniAdrcControllerNode::EulertoQ(const Eigen::Vector3d &rpy, Eigen::Quaterniond &Q)
{
  double cosPhi_2 = double(cos(rpy(0) / double(2.0)));
  double cosTheta_2 = double(cos(rpy(1) / double(2.0)));
  double cosPsi_2 = double(cos(rpy(2) / double(2.0)));
  double sinPhi_2 = double(sin(rpy(0) / double(2.0)));
  double sinTheta_2 = double(sin(rpy(1) / double(2.0)));
  double sinPsi_2 = double(sin(rpy(2) / double(2.0)));
  Q.w() = cosPhi_2 * cosTheta_2 * cosPsi_2 +
          sinPhi_2 * sinTheta_2 * sinPsi_2;
  Q.x() = sinPhi_2 * cosTheta_2 * cosPsi_2 -
          cosPhi_2 * sinTheta_2 * sinPsi_2;
  Q.y() = cosPhi_2 * sinTheta_2 * cosPsi_2 +
          sinPhi_2 * cosTheta_2 * sinPsi_2;
  Q.z() = cosPhi_2 * cosTheta_2 * sinPsi_2 -
          sinPhi_2 * sinTheta_2 * cosPsi_2;
  Q.normalize();
}

void OmniAdrcControllerNode::OdometryCallback(const nav_msgs::Odometry &odometry)
{
  // ///////gazebo nav_msgs::Odometry odometry///////////  // Eigen::Quaterniond Q;
  ROS_INFO_ONCE("get odometry message");
  _odometry_got = true;

  double secs = ros::Time::now().toSec();
  double dt = secs - _sensors_lastsecs;
  _sensors_lastsecs = secs;
  // ROS_INFO("sensors dt:  %f", dt);
  _state_q.w() = odometry.pose.pose.orientation.w;
  _state_q.x() = odometry.pose.pose.orientation.x;
  _state_q.y() = odometry.pose.pose.orientation.y;
  _state_q.z() = odometry.pose.pose.orientation.z;
  _state_q.normalize();

  _state_pos(0) = odometry.pose.pose.position.x;
  _state_pos(1) = odometry.pose.pose.position.y;
  _state_pos(2) = odometry.pose.pose.position.z;
  _state_rates(0) = odometry.twist.twist.linear.x;
  _state_rates(1) = odometry.twist.twist.linear.y;
  _state_rates(2) = odometry.twist.twist.linear.z;
  _state_rates = _state_q.toRotationMatrix()*_state_rates;
  _state_omega(0) = odometry.twist.twist.angular.x;
  _state_omega(1) = odometry.twist.twist.angular.y;
  _state_omega(2) = odometry.twist.twist.angular.z;

  QtoEuler(_state_rpy, _state_q);
  std_msgs::Float64 msg;
  msg.data = _state_rpy(0) / M_PI * 180;
  _state_r_pub.publish(msg);
  msg.data = _state_rpy(1) / M_PI * 180;
  _state_p_pub.publish(msg);
  msg.data = _state_rpy(2) / M_PI * 180;
  _state_y_pub.publish(msg);
}
void OmniAdrcControllerNode::TimedControlCallback(const ros::TimerEvent &event)
{
  double secs = ros::Time::now().toSec();
  double dt = secs - _control_lastsecs;
  _control_lastsecs = secs;
  // ROS_INFO("control dt:  %f", dt);
  if (_odometry_got)
  {
    EulerBasedControl(dt);
    ControlAllocation();
  }
}
void OmniAdrcControllerNode::EulerBasedControl(double dt)
{
    //位置控制用串级adrc
  _F_e << _adrc_x.CascadeControl(_command_pos(0), _state_pos(0), _state_rates(0)),
      _adrc_y.CascadeControl(_command_pos(1), _state_pos(1), _state_rates(1)),
      _adrc_z.CascadeControl(_command_pos(2), _state_pos(2), _state_rates(2));

  if (_F_e.norm() > _mass * 9.8)
  {
    _F_e *= _mass * 9.8 / _F_e.norm();
  }
  _F_e(2) += _mass * 9.8;

 //角度环用基于四元素的p控制   角速度环 一阶LADRC
  // Eigen::Vector3d command_omega(0, 0, 0);
  // command_omega << _attitude_gain(0) * (_command_rpy(0) - _state_rpy(0)),
  //     _attitude_gain(1) * (_command_rpy(1) - _state_rpy(1)),
  //     _attitude_gain(2) * (_command_rpy(2) - _state_rpy(2));
  Eigen::Vector3d command_omega(0, 0, 0);
  _state_q.normalize();
  _command_q.normalize();
  Eigen::Quaterniond error_q = _state_q.conjugate() * _command_q;
  error_q.normalize();
  command_omega = 2.0f / 0.5 * Sgn(error_q.w()) * error_q.vec();

  _M_b << _adrc_roll.ComputeControl(command_omega(0), _state_omega(0)),
      _adrc_pitch.ComputeControl(command_omega(1), _state_omega(1)),
      _adrc_yaw.ComputeControl(command_omega(2), _state_omega(2));


  if (_M_b.norm() >4)
  {
    _M_b *= 4 / _M_b.norm();
  }
  // std_msgs::double32 msg;

  // msg.data = _adrc_roll.GetLESOZ1() / M_PI * 180;
  // _roll_eso_z1_pub.publish(msg);
  // msg.data = _adrc_roll.GetLESOZ2() / M_PI * 180;
  // _roll_eso_z2_pub.publish(msg);

}


void OmniAdrcControllerNode::ControlAllocation()
{
  Eigen::Vector3d F_b;
  _state_q.normalize();
  Eigen::Matrix3d R_b_e = _state_q.toRotationMatrix();
  F_b = R_b_e.transpose() * _F_e;
  Eigen::Matrix<double, 6, 1> FM_b;
  FM_b << F_b, _M_b;
  Eigen::Matrix<double, 8, 1> VirtualControl;
  VirtualControl = _allocationMatrix * FM_b;
  double w0, w1, w2, w3, alpha0, alpha1, alpha2, alpha3;
  w0 = sqrt(sqrt(VirtualControl(0) * VirtualControl(0) + VirtualControl(1) * VirtualControl(1)));
  alpha0 = atan2(VirtualControl(0), VirtualControl(1));
  w1 = sqrt(sqrt(VirtualControl(2) * VirtualControl(2) + VirtualControl(3) * VirtualControl(3)));
  alpha1 = atan2(VirtualControl(2), VirtualControl(3));
  w2 = sqrt(sqrt(VirtualControl(4) * VirtualControl(4) + VirtualControl(5) * VirtualControl(5)));
  alpha2 = atan2(VirtualControl(4), VirtualControl(5));
  w3 = sqrt(sqrt(VirtualControl(6) * VirtualControl(6) + VirtualControl(7) * VirtualControl(7)));
  alpha3 = atan2(VirtualControl(6), VirtualControl(7));

  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
  actuator_msg->angular_velocities.clear();
  actuator_msg->angular_velocities.push_back(w0);
  actuator_msg->angular_velocities.push_back(w1);
  actuator_msg->angular_velocities.push_back(w2);
  actuator_msg->angular_velocities.push_back(w3);

  _actuator_motor_speed_pub.publish(actuator_msg);

  actuator_msg->angular_velocities.clear();
  actuator_msg->angular_velocities.push_back(alpha0);
  actuator_msg->angular_velocities.push_back(alpha1);
  actuator_msg->angular_velocities.push_back(alpha2);
  actuator_msg->angular_velocities.push_back(alpha3);

  _actuator_servo_position_pub.publish(actuator_msg);
}
void OmniAdrcControllerNode::CommandPoseCallback(const vfly::vfly_pose &pose)
{
  _command_pos(0) = pose.x;
  _command_pos(1) = pose.y;
  _command_pos(2) = pose.z;
  _command_rpy(0) = pose.roll / 180 * M_PI;
  _command_rpy(1) = pose.pitch / 180 * M_PI;
  _command_rpy(2) = pose.yaw / 180 * M_PI;

  EulertoQ(_command_rpy, _command_q);

  Eigen::Vector3d rpy;
  QtoEuler(rpy, _command_q);
  ROS_INFO("command_rpy: %f %f %f", rpy(0) * 180 / M_PI, rpy(1) * 180 / M_PI, rpy(2) * 180 / M_PI);

 
}

void OmniAdrcControllerNode::Constraint(double &val, double limit)
{
  if (std::abs(val) > limit)
  {
    val = val * limit / std::abs(val);
  }
}

double OmniAdrcControllerNode::Sgn(double val)
{
  if (val >= 0.0f)
    return 1.0f;
  else
    return -1.0f;
}

inline void OmniAdrcControllerNode::GetRosParameter(const ros::NodeHandle &nh, const std::string &key,
                                                      const double &default_value, double *value)
{
  ROS_ASSERT(value != nullptr);
  bool have_parameter = nh.getParam(key, *value);
  if (!have_parameter)
  {
    ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace() << "/" << key
                                                            << ", setting to default: " << default_value);
    *value = default_value;
  }
}

Eigen::MatrixXd OmniAdrcControllerNode::Pinv(Eigen::MatrixXd A)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV); // M=USV*
  double Pinvtoler = 1.e-8;                                                             // tolerance
  int row = A.rows();
  int col = A.cols();
  int k = std::min(row, col);
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col, row);
  Eigen::MatrixXd singularValues_inv = svd.singularValues(); //奇异值
  Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
  for (long i = 0; i < k; ++i)
  {
    if (singularValues_inv(i) > Pinvtoler)
      singularValues_inv(i) = 1.0 / singularValues_inv(i);
    else
      singularValues_inv(i) = 0;
  }
  for (long i = 0; i < k; ++i)
  {
    singularValues_inv_mat(i, i) = singularValues_inv(i);
  }
  X = (svd.matrixV()) * (singularValues_inv_mat) * (svd.matrixU().transpose()); // X=VS+U*

  return X;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "omni_adrc_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  OmniAdrcControllerNode omni_adrc_controller_node(nh, private_nh);
  ros::spin();
  return 0;
}
