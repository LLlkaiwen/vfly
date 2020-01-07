#include "omni_pid_controller_node.h"

OmniPidControllerNode::OmniPidControllerNode(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh)
    : _nh(nh)
    , _private_nh(private_nh)
    , _position_gain(3.0f, 3.0f, 3.0f)
    , _attitude_gain(3.0f, 3.0f, 3.0f)
    , _state_q(1, 0, 0, 0)
    , _state_rpy(0.0f, 0.0f, 0.0f)
    , _state_pos(0.0f, 0.0f, 0.0f)
    , _state_omega(0.0f, 0.0f, 0.0f)
    , _state_rates(0.0f, 0.0f, 0.0f)
    , _command_rpy(0.0f, 0.0f, 0.0f)
    , _command_pos(0.0f, 0.0f, 3.0f)
    , _command_q(1.0f, 0.0f, 0.0f, 0.0f)
    , _F_e(0.0f, 0.0f, 0.0f)
    , _M_b(0.0f, 0.0f, 0.0f)
    , _odometry_got(false)
{
  InitializeParams();

  _actuator_servo_position_pub = _nh.advertise<mav_msgs::Actuators>("gazebo/command/servo_position", 1);

  _actuator_motor_speed_pub = _nh.advertise<mav_msgs::Actuators>("gazebo/command/motor_speed", 1);

  _state_r_pub = _nh.advertise<std_msgs::Float32>("state/roll", 1);

  _state_p_pub = _nh.advertise<std_msgs::Float32>("state/pitch", 1);

  _state_y_pub = _nh.advertise<std_msgs::Float32>("state/yaw", 1);

  _sensor_odemetry_sub = _nh.subscribe("ground_truth/odometry", 1, &OmniPidControllerNode::OdometryCallback, this);

  _command_pose_sub = _nh.subscribe("command/pose", 1, &OmniPidControllerNode::CommandPosCallback, this);
  _control_timer = _nh.createTimer(ros::Duration(0.004), &OmniPidControllerNode::TimedControlCallback, this);

  _control_lastsecs = _sensors_lastsecs = ros::Time::now().toSec();
}

OmniPidControllerNode::~OmniPidControllerNode()
{
}

void OmniPidControllerNode::InitializeParams()
{
  float gain, kp, ki, kd, integral_limit, output_limit;
  //从 rosparam 读取控制器参数 初始化控制器
  GetRosParameter(_private_nh, "position_gain/x", 3, &gain);
  _position_gain(0) = gain;
  GetRosParameter(_private_nh, "position_gain/y", 3, &gain);
  _position_gain(1) = gain;
  GetRosParameter(_private_nh, "position_gain/z", 3, &gain);
  _position_gain(2) = gain;

  GetRosParameter(_private_nh, "attitude_gain/x", 3, &gain);
  _attitude_gain(0) = gain;
  GetRosParameter(_private_nh, "attitude_gain/y", 3, &gain);
  _attitude_gain(1) = gain;
  GetRosParameter(_private_nh, "attitude_gain/z", 3, &gain);
  _attitude_gain(2) = gain;

  GetRosParameter(_private_nh, "pid_u/kp", 1, &kp);
  GetRosParameter(_private_nh, "pid_u/ki", 0.01, &ki);
  GetRosParameter(_private_nh, "pid_u/kd", 0.55, &kd);
  GetRosParameter(_private_nh, "pid_u/integral_limit", 2, &integral_limit);
  GetRosParameter(_private_nh, "pid_u/output_limit", 1, &output_limit);
  pid_init(&_pid_u, PID_MODE_DERIVATIV_CALC, 0.001f);
  pid_set_parameters(&_pid_u, kp, ki, kd, integral_limit, output_limit);

  GetRosParameter(_private_nh, "pid_v/kp", 1, &kp);
  GetRosParameter(_private_nh, "pid_v/ki", 0.01, &ki);
  GetRosParameter(_private_nh, "pid_v/kd", 0.55, &kd);
  GetRosParameter(_private_nh, "pid_v/integral_limit", 2, &integral_limit);
  GetRosParameter(_private_nh, "pid_v/output_limit", 1, &output_limit);
  pid_init(&_pid_v, PID_MODE_DERIVATIV_CALC, 0.001f);
  pid_set_parameters(&_pid_v, kp, ki, kd, integral_limit, output_limit);

  GetRosParameter(_private_nh, "pid_w/kp", 1, &kp);
  GetRosParameter(_private_nh, "pid_w/ki", 0.01, &ki);
  GetRosParameter(_private_nh, "pid_w/kd", 0.55, &kd);
  GetRosParameter(_private_nh, "pid_w/integral_limit", 2, &integral_limit);
  GetRosParameter(_private_nh, "pid_w/output_limit", 1, &output_limit);
  pid_init(&_pid_w, PID_MODE_DERIVATIV_CALC, 0.001f);
  pid_set_parameters(&_pid_w, kp, ki, kd, integral_limit, output_limit);

  GetRosParameter(_private_nh, "pid_p/kp", 1.8, &kp);
  GetRosParameter(_private_nh, "pid_p/ki", 0.025, &ki);
  GetRosParameter(_private_nh, "pid_p/kd", 0.88, &kd);
  GetRosParameter(_private_nh, "pid_p/integral_limit", 2, &integral_limit);
  GetRosParameter(_private_nh, "pid_p/output_limit", 1, &output_limit);
  pid_init(&_pid_p, PID_MODE_DERIVATIV_CALC, 0.001f);
  pid_set_parameters(&_pid_p, kp, ki, kd, integral_limit, output_limit);

  GetRosParameter(_private_nh, "pid_q/kp", 1.8, &kp);
  GetRosParameter(_private_nh, "pid_q/ki", 0.025, &ki);
  GetRosParameter(_private_nh, "pid_q/kd", 0.88, &kd);
  GetRosParameter(_private_nh, "pid_q/integral_limit", 2, &integral_limit);
  GetRosParameter(_private_nh, "pid_q/output_limit", 1, &output_limit);
  pid_init(&_pid_q, PID_MODE_DERIVATIV_CALC, 0.001f);
  pid_set_parameters(&_pid_q, kp, ki, kd, integral_limit, output_limit);

  GetRosParameter(_private_nh, "pid_r/kp", 1.8, &kp);
  GetRosParameter(_private_nh, "pid_r/ki", 0.025, &ki);
  GetRosParameter(_private_nh, "pid_r/kd", 0.88, &kd);
  GetRosParameter(_private_nh, "pid_r/integral_limit", 2, &integral_limit);
  GetRosParameter(_private_nh, "pid_r/output_limit", 1, &output_limit);
  pid_init(&_pid_r, PID_MODE_DERIVATIV_CALC, 0.001f);
  pid_set_parameters(&_pid_r, kp, ki, kd, integral_limit, output_limit);

  float kf, km, l;
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

  _allocationMatrix = Pinv(angular_theta_to_fm_b);
  std::cout << "FM_b_to_actuator allocation:\n"
            << _allocationMatrix << std::endl;
}

void OmniPidControllerNode::OdometryCallback(/*geometry_msgs::Pose odometry*/ nav_msgs::Odometry odometry)
{
  // ///////gazebo nav_msgs::Odometry odometry///////////  // Eigen::Quaterniond Q;
  ROS_INFO_ONCE("get odometry message");
  _odometry_got = true;
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
  _state_rates = _state_q.toRotationMatrix()*_state_rates; //从机体系旋转到导航系
  _state_omega(0) = odometry.twist.twist.angular.x;
  _state_omega(1) = odometry.twist.twist.angular.y;
  _state_omega(2) = odometry.twist.twist.angular.z;
  double secs = ros::Time::now().toSec();
  double dt = secs - _sensors_lastsecs;
  _sensors_lastsecs = secs;

  QtoEuler(_state_rpy, _state_q);

  std_msgs::Float32 msg;
  msg.data = _state_rpy(0) / M_PI * 180;
  _state_r_pub.publish(msg);
  msg.data = _state_rpy(1) / M_PI * 180;
  _state_p_pub.publish(msg);
  msg.data = _state_rpy(2) / M_PI * 180;
  _state_y_pub.publish(msg);

}
void OmniPidControllerNode::TimedControlCallback(const ros::TimerEvent &event)
{
  double secs = ros::Time::now().toSec();
  double dt = secs - _control_lastsecs;
  _control_lastsecs = secs;
  if (_odometry_got)
  {
    // EulerBasedControl(dt);
    QuaternionBasedControl(dt);
    ControlAllocation();
  }
}

void OmniPidControllerNode::EulerBasedControl(double dt)
{
////////////位置控制 双环PID///////////////////
  Eigen::Vector3d command_rates(0, 0, 0);
  command_rates << _position_gain(0) * (_command_pos(0) - _state_pos(0)),
      _position_gain(1) * (_command_pos(1) - _state_pos(1)),
      _position_gain(2) * (_command_pos(2) - _state_pos(2));
  Eigen::Vector3d desired_accel(0, 0, 0);
  desired_accel << pid_calculate(&_pid_u, command_rates(0), _state_rates(0), 0, dt),
      pid_calculate(&_pid_v, command_rates(1), _state_rates(1), 0, dt),
      pid_calculate(&_pid_w, command_rates(2), _state_rates(2), 0, dt);
  if (desired_accel.norm() > 1)
  {
    desired_accel /= desired_accel.norm();
  }
  _F_e = desired_accel * _mass * 9.8;
  _F_e(2) += _mass * 9.8;

  ///////////////姿态控制 双环PID///////////
  Eigen::Vector3d command_omega(0, 0, 0);
  command_omega << _attitude_gain(0) * (_command_rpy(0) - _state_rpy(0)),
      _attitude_gain(1) * (_command_rpy(1) - _state_rpy(1)),
      _attitude_gain(2) * (_command_rpy(2) - _state_rpy(2));

  Eigen::Vector3d desired_angular_accel(0, 0, 0);
  desired_angular_accel << pid_calculate(&_pid_p, command_omega(0), _state_omega(0), 0, dt),
      pid_calculate(&_pid_q, command_omega(1), _state_omega(1), 0, dt),
      pid_calculate(&_pid_r, command_omega(2), _state_omega(2), 0, dt);


  // Eigen::Vector3d IOMEGA ;
  // IOMEGA << _state_omega(0) * _Ix, _state_omega(1) * _Iy, _state_omega(2) * _Iz;
  // _M_b = _state_omega.cross(IOMEGA);
  // _M_b += desired_angular_accel * 4;
  if (desired_angular_accel.norm() > 1)
  {
    desired_angular_accel /= desired_angular_accel.norm();
  }
  _M_b = desired_angular_accel * 4;
}

void OmniPidControllerNode::QuaternionBasedControl(double dt)
{
  ////////////位置控制 双环PID///////////////////
  Eigen::Vector3d command_rates(0, 0, 0);
  command_rates << _position_gain(0) * (_command_pos(0) - _state_pos(0)),
      _position_gain(1) * (_command_pos(1) - _state_pos(1)),
      _position_gain(2) * (_command_pos(2) - _state_pos(2));

  Eigen::Vector3d desired_accel(0, 0, 0);
  desired_accel << pid_calculate(&_pid_u, command_rates(0), _state_rates(0), 0, dt),
      pid_calculate(&_pid_v, command_rates(1), _state_rates(1), 0, dt),
      pid_calculate(&_pid_w, command_rates(2), _state_rates(2), 0, dt);

  _F_e = desired_accel * _mass * 9.8;
  _F_e(2) += _mass * 9.8;

  ///////////////姿态控制 双环PID///////////

  Eigen::Vector3d command_omega(0, 0, 0);
  _state_q.normalize();
  _command_q.normalize();
  Eigen::Quaterniond error_q = _state_q.conjugate() * _command_q;
  command_omega = 2.0f / 0.275 * Sgn(error_q.w()) * error_q.vec();

  Eigen::Vector3d desired_angular_accel(0, 0, 0);
  desired_angular_accel << pid_calculate(&_pid_p, command_omega(0), _state_omega(0), 0, dt),
      pid_calculate(&_pid_q, command_omega(1), _state_omega(1), 0, dt),
      pid_calculate(&_pid_r, command_omega(2), _state_omega(2), 0, dt);


  // Eigen::Vector3d IOMEGA ;
  // IOMEGA << _state_omega(0) * _Ix, _state_omega(1) * _Iy, _state_omega(2) * _Iz;
  // _M_b = _state_omega.cross(IOMEGA);
  // _M_b += desired_angular_accel * 4;
   if (desired_angular_accel.norm() > 1)
  {
    desired_angular_accel /= desired_angular_accel.norm();
  }
  _M_b = desired_angular_accel * 4;
}

void OmniPidControllerNode::ControlAllocation()
{
  Eigen::Vector3d F_b;
  _state_q.normalize();
  Eigen::Matrix3d R_b_e = _state_q.toRotationMatrix();
  Eigen::Matrix3d R_e_b  = R_b_e.transpose();
  F_b = R_e_b *  _F_e; 
  //////////////通过控制分配将机体系期望力和力矩转化为电机转速和倾转角
  Eigen::Matrix<double, 6, 1> FM_b;
  FM_b << F_b, _M_b;
  Eigen::Matrix<double, 8, 1> VirtualControl;
  VirtualControl = _allocationMatrix * FM_b;
  float w0, w1, w2, w3, alpha0, alpha1, alpha2, alpha3;
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
void OmniPidControllerNode::CommandPosCallback(vfly::vfly_pose pose)
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

void OmniPidControllerNode::Constraint(float &val, float limit)
{
  if (std::abs(val) > limit)
  {
    val = val * limit / std::abs(val);
  }
}

void OmniPidControllerNode::QtoEuler(Eigen::Vector3d &rpy, const Eigen::Quaterniond &Q)
{
  rpy(0) = atan2(2 * (Q.w() * Q.x() + Q.y() * Q.z()), 1 - 2 * (Q.x() * Q.x() + Q.y() * Q.y()));
  rpy(1) = asin(2 * (Q.w() * Q.y() - Q.x() * Q.z()));
  rpy(2) = atan2(2 * (Q.w() * Q.z() + Q.y() * Q.x()), 1 - 2 * (Q.z() * Q.z() + Q.y() * Q.y()));
}

void OmniPidControllerNode::EulertoQ(const Eigen::Vector3d &rpy, Eigen::Quaterniond &Q)
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

double OmniPidControllerNode::Sgn(double val)
{
  if (val >= 0.0f)
    return 1.0f;
  else
    return -1.0f;
}

inline void OmniPidControllerNode::GetRosParameter(const ros::NodeHandle &nh, const std::string &key,
                                                     const float &default_value, float *value)
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

Eigen::MatrixXd OmniPidControllerNode::Pinv(Eigen::MatrixXd A)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV); // M=USV*
  double Pinvtoler = 1.e-8;                                                            // tolerance
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
  ros::init(argc, argv, "omni_pid_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  OmniPidControllerNode omni_pid_controller_node(nh, private_nh);
  ros::spin();
  return 0;
}
