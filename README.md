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
- N3飞控版本：v1.7.6.0

- 遥控器设置
  - U，设置为REV，设置为模式切换，即 P\S\A （P为定点，A模式下才进入sdk控制）
  - 起落架通道，不REV，设置为自动轨迹trigger
  - 具体看截图
- N3与机载电脑连接
  - API接口，从左至右，空着、GND、RX、TX
- SDK设置：启动API控制
- 无人机姿态环抖动，调大动力带宽
- 使用N3初始设置绑定的DJI账号中生成app_id和enc_key，加入djiros.launch文件中
  - 首次启动需连接电脑激活（启动launch文件2次）
 
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

ego_planner_node.cpp 是入口程序，声明并初始化了EGOReplanFSM类
ego_replan_fsm.cpp 是EGOReplanFSM类
  - 声明并初始化了EGOPlannerManager类
  - 定时器exec_timer_ 状态机循环 0.01秒
  - 定时器safety_timer_ 安全检测 0.05秒
  - 订阅里程计
  - 发布B样条<ego_planner::Bspline>（调用EGOPlannerManager类）
planner_manager.cpp是EGOPlannerManager类
  - 声明并初始化了GridMap类
  - 声明并初始化了BsplineOptimizer类
  - 在BsplineOptimizer类中初始化了gridmap地图
  - 在BsplineOptimizer类中初始化了a_star_
bspline_optimizer.cpp是BsplineOptimizer类
  - 调用a_star_规划全局路径，优化路径
path_serching.cpp是AStar类
grid_map.cpp是GridMap类（他这里是增量式的建图吗？）
  - 订阅了深度
  - 订阅了点云、里程计
  - 发布了占据地图、膨胀后的占据地图
  - 调用了raycast.cpp

traj_server.cpp
  - 订阅B样条<ego_planner::Bspline>
  - 发布控制指令<quadrotor_msgs::PositionCommand>，100Hz

traj_utils中polynomial_traj.cpp是计算多项式轨迹的
traj_utils中planning_visualization.cpp是用于rviz显示路径、目标点等显示的
