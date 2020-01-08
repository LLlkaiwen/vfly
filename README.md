VFly
============

VFly 是基于Gazebo的 推力矢量可倾转多旋翼仿真ROS功能包。  
VFly  is a ROS package of thrust-vectored MAV with tiltable rotors gazebo simulator. 

功能包提供了可倾转四旋翼模型，全向运动PID和ADRC控制器和相应的launch文件。  
This package provides a model  of quadrotor with tiltable rotors, example omni-direction controller based on PID and ADRC, and example launch files. 

如果你在论文中有用到本仿真包请引用：  
If you are using this simulator within the research for your publication, please cite:

```
卢凯文,杨忠,张秋雁,许昌亮,徐浩,徐向荣.推力矢量可倾转四旋翼自抗扰飞行控制方法[J/OL].控制理论与应用  
LU Kaiwen, YANG Zhong, ZHANG Qiuyan. Active disturbance rejection flight control method for thrust-vectored quadrotor with tiltable rotors. Control Theory & Applications
```

安装向导
----------

1. 本功能包使用了[RotorS](https://github.com/ethz-asl/rotors_simulator)的部分Gazebo插件，在使用之前需要安装RotorS。  

    > **Note** 如果在安装RotorS 过程中遇到问题，欢迎交流.  

2. 获取功能包和依赖

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/LLlkaiwen/vfly.git
    $ git clone https://github.com/LLlkaiwen/adrc_control.git
    ```
3. 构建你的工作空间

   ```
   $ cd ~/catkin_ws/
   $ catkin init  # If you haven't done this before.
   $ catkin build / catkin_make 
   ```

使用向导
-----------

1. 启动Gazebo仿真("x"构型的VFly)

    ```
    $ roslaunch vfly mav.launch mav_name:=vfly type:=x
    ```

2. 启动全向控制器

    ```
    $ roslaunch vfly omni_pid_controller.launch mav_name:=vfly type:=x
    ```
    > **Note** 控制器会控制VFly以水平姿态悬停在3m的高度. 

    当然，你也可以启动ADRC全向控制器 (PID和ADRC二选一)

    ```
    $ roslaunch vfly omni_adrc_controller.launch mav_name:=vfly type:=x
    ```
3. 发送控制指令

    * 水平飞行(以当前状态水平悬停在3m处为例)

    ```
    rostopic pub -1 /vfly/command/pose vfly/vfly_pose "header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
    x: 3.0
    y: 3.0
    z: 3.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0" 
    ```

    > **Note** 你可以在输入rostopic pub -1 /vfly/command/pose后，按两次Tab键自动补全后面的内容，然后修改其中的姿态和位置期望。

    * 定点旋转(以当前状态水平悬停在3m处为例)
    
    ```
    rostopic pub -1 /vfly/command/pose vfly/vfly_pose "header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
    x: 0.0
    y: 0.0
    z: 3.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0" 
    ```
    > **Note** 你可以同时给定姿态和位置期望，因为可倾转多旋翼有别于常规多旋翼，可以同时控制姿态和位置。

4. 查看位置姿态响应曲线

    ```
    $ rqt_plot /vfly/ground_truth/position/point /vfly/state/roll /vfly/state/pitch /vfly/state/yaw
    ```
