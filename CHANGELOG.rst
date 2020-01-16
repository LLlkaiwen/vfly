^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vfly
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2020-01-07)
------------------
完成vfly gazebo仿真环境搭建
整定pid控制器和adrc控制器

omni_pid_controller:
位置控制 ：外环p+内环pid
姿态控制： 外环p(可选基于欧拉角/四元数)+内环pid

omni_adrc_controller:
位置控制 ：串级adrc
姿态控制： 外环p(可选基于欧拉角/四元数)+内环一阶ladrc

1.0.1(2020-01-15)
-----------------------
添加example程序
rotation_on_fixed_spot_example.cpp
生成位置姿态期望，实现VFly定点360度可控空翻。
move_with_zero_attitude_example.cpp
生成位置姿态期望，实现VFly零姿态追踪“8”字形轨迹