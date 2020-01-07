^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vfly
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2020-01-07)
------------------
完成vfly gazebo仿真环境搭建
整定pid控制器和adrc控制器

omni_pid_controller:
位置控制 ：外环p+内环pid
姿态控制： 外环p(可选基于欧拉角/四元数)+内环pid

omni_adrc_controller:
位置控制 ：串级adrc
姿态控制： 外环p(可选基于欧拉角/四元数)+内环一阶ladrc