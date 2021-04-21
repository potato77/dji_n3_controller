# dji_n3_controller

### 快速上手

- 下载代码
  ```c
  git clone https://github.com/potato77/dji_n3_controller.git
  ```
  
- 安装Onboard-SDK
  ```c

  cd dji_n3_controller/src/Onboard-SDK
  mkdir build
  cd build
  cmake ..
  sudo make -j7 install
  ```

- 编译整个项目
  ```
  cd dji_n3_controller/
  catkin_make
  ```

- 使用
  ```
  ./all.sh
  ```

### 使用说明

### 代码说明

- Onboard-SDK是DJI官方提供的3.7版本
- controller/djiros是港科大改动过的Onboard-SDK-ROS，部分控制话题被修改
- controller/n3ctrl是控制状态机+位置环控制器代码
  ```
   ## 订阅以下话题
   ~imu  : [sensor_msgs/IMU]               IMU message from djiros.
   ~odom  : [nav_msgs/Odometry]              里程计消息
   ~joy   : [sensor_msgs/Joy]              RC message form djiros
   ~cmd : [quadrotor_msgs/PositionCommand] Position command message from the planner.
    ## 发布以下话题
    ~traj_start_trigger  : [geometry_msgs/PoseStamped]        A trigger message is sent when enter command mode.
    ~desire_pose : [geometry_msgs/PoseStamped]     The desired pose of the controller.
     ~ctrl : [sensor_msgs/Joy] The output of the control signal. The axes are the roll, pitch, thrust, yaw or yaw rate, thrust mode (if thrust mode > 0, thrust = 0~100%; if mode < 0, thrust = -? m/s ~ +? m/s), yaw mode (if yaw_mode > 0, axes[3] = yaw; if yaw_mode < 0, axes[3] = yaw_rate) respectively. The roll, pitch and yaw or yaw rate are in FRD frame. 
     ## 参数文件
     $(find n3ctrl)/config/ctrl_param_$(arg uavname).yaml
  ```
- ego-planner是规划器
  - 不给定传感器输入时，建立全可通行地图，则变成纯轨迹规划器

