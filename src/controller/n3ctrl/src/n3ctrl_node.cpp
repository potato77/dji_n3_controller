#include <ros/ros.h>
#include "N3CtrlFSM.h"

#include <quadrotor_msgs/SO3Command.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <signal.h>

#include <n3ctrl/ControllerDebug.h>
N3CtrlFSM* pFSM;

void mySigintHandler(int sig) {
    pFSM->stateVisualizer.publish_led_vis(ros::Time::now(), "null");
    ROS_ERROR("[N3Ctrl] exit...");
    ros::shutdown();
}


int main(int argc, char* argv[]) {
    ros::init(argc, argv, "n3ctrl");
    ros::NodeHandle nh("~");

    // 不懂
    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    // 参数类，定义在N3CtrlParam
    Parameter_t param;
    // 控制器类，定义在controller，并使用参数进行初始化`
    Controller controller(param);
    // 悬停油门卡尔曼滤波？？？，定义在hovthrkf
    HovThrKF hov_thr_kf(param);
    // 状态机类，定义在N3CtrlFSM，注：N3CtrlFSM_state是函数api，N3CtrlFSM_control也是api函数
    // fsm中集合使用了controller，hov_thr_kf，input
    N3CtrlFSM fsm(param, controller, hov_thr_kf);
    pFSM = &fsm;

    // 参数初始化，从ros节点中读取参数
    param.config_from_ros_handle(nh);
    // 根据质量及悬停油门计算full thrust
    param.init();
    // 卡尔曼滤波初始化
    fsm.hov_thr_kf.init();
    // 设置悬停油门
    fsm.hov_thr_kf.set_hov_thr(param.hov_percent);
    // 这个是false
    if (param.hover.set_hov_percent_to_zero) {
        // set to zero for debug
        fsm.hov_thr_kf.set_hov_thr(0.0);
    }
    // 这两个值是相等的，应该是检查打印
    // full_thrust = mass * gra / hov_percent;
    ROS_INFO("Initial value for hov_thr set to %.2f/%.2f",
             fsm.hov_thr_kf.get_hov_thr(),
             param.mass * param.gra / param.full_thrust);
    // use_hov_percent_kf = true
    ROS_INFO("Hovering thrust kalman filter is %s.",
             param.hover.use_hov_percent_kf ? "used" : "NOT used");

    bool skip_wait_for_rc = false;
    // 默认是实际飞行模式
    if (param.work_mode.compare("simulation") == 0) {
        fsm.set_work_mode(N3CtrlFSM::SIMULATION);
        skip_wait_for_rc = true;
    } else if (param.work_mode.compare("sim_without_rc") == 0) {
        fsm.set_work_mode(N3CtrlFSM::SIM_WITHOUT_RC);
        skip_wait_for_rc = true;
    } else {
        fsm.set_work_mode(N3CtrlFSM::REALTIME);
    }

    // 默认是feedback
    if (param.js_ctrl_mode.compare("raw") == 0) {
        fsm.set_js_ctrl_mode(N3CtrlFSM::JS_CTRL_MODE_RAW);
    } else {
        fsm.set_js_ctrl_mode(N3CtrlFSM::JS_CTRL_MODE_FEEDBACK);
    }

    //  设置当前默认的遥控器模式及脚架通道，此处和接收遥控器指令相关
    fsm.rc_data.set_default_mode(std::string("manual"));
    fsm.rc_data.set_default_mode(std::string("noapi"));

    // 配置controller参数: 即hover_gain中的13个Kp参数
    fsm.controller.config();

    // 订阅遥控器指令，dji_sdk发布，回调在input.cpp
    // 一个是做有rc的定点控制，一个是做模式切换和轨迹trigger
    // 参考资料：http://wiki.ros.org/dji_sdk
    ros::Subscriber joy_sub =
        nh.subscribe<sensor_msgs::Joy>("joy",
                                       1000,
                                       boost::bind(&RC_Data_t::feed, &fsm.rc_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    // 订阅里程计消息，icon_odom发布，回调在input.cpp
    // 提取了位置、速度、姿态、角速度，此处暂时不确定是否使用了角速度
    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         1000,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());
    
    // 订阅IMU数据，dji_sdk发布，回调在input.cpp
    // 提取了角速度、线加速度、姿态
    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("imu",
                                       1000,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    // 新增
    ros::Subscriber cmd_point_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
        "point_cmd",
        1000,
        boost::bind(&Cmd_point_Data_t::feed, &fsm.point_data, _1),
        ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());


    // 订阅控制指令，上层发布，回调在input.cpp
    // 期望位置、速度、加速度、期望偏航角、轨迹ID
    ros::Subscriber cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>(
        "cmd",
        1000,
        boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
        ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());

    // 暂不清楚这个是做什么用的
    ros::Subscriber idle_sub = nh.subscribe<geometry_msgs::Vector3Stamped>(
        "idling",
        1000,
        boost::bind(&Idling_Data_t::feed, &fsm.idling_data, _1),
        ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay());

    // 暂不清楚这个是做什么用的，js指joystick
    ros::Subscriber enter_js_sub =
        nh.subscribe<std_msgs::Header>("enter_js",
                                       1000,
                                       boost::bind(&Trigger_Data_t::feed, &fsm.trigger_data, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    // 发布底层控制指令至dji_sdk
    fsm.controller.ctrl_pub = nh.advertise<sensor_msgs::Joy>("ctrl", 10);

    // fsm.controller.ctrl_so3_pub	=
    // 	nh.advertise<quadrotor_msgs::SO3Command>("ctrl_so3", 10);

    // 以下好多都没真正发布
    // 发布 so3控制指令，应该用处不大，是用于仿真和调试
    fsm.controller.ctrl_so3_attitude_pub =
        nh.advertise<geometry_msgs::QuaternionStamped>("ctrl_so3/attitude", 10);

    fsm.controller.ctrl_so3_thrust_pub =
        nh.advertise<geometry_msgs::WrenchStamped>("ctrl_so3/thrust", 10);

    // 可视化
    fsm.controller.ctrl_vis_pub = nh.advertise<sensor_msgs::Imu>("ctrl_vis", 10);

    // debug相关
    fsm.controller.ctrl_dbg_pub = nh.advertise<std_msgs::Header>("ctrl_dbg/info", 10);

    fsm.controller.ctrl_val_dbg_pub = nh.advertise<n3ctrl::ControllerDebug>("ctrl_dbg/value", 10);

    fsm.controller.ctrl_dbg_p_pub = nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_dbg/p", 10);

    fsm.controller.ctrl_dbg_v_pub = nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_dbg/v", 10);

    fsm.controller.ctrl_dbg_a_pub = nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_dbg/a", 10);

    fsm.controller.ctrl_dbg_att_des_pub =
        nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_dbg/att_des", 10);

    fsm.controller.ctrl_dbg_att_real_pub =
        nh.advertise<geometry_msgs::Vector3Stamped>("ctrl_dbg/att_real", 10);

    fsm.hov_thr_kf.hov_thr_pub = nh.advertise<std_msgs::Float64>("hov_thr", 10);

    // 真正发布了!
    fsm.des_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("desire_pose", 10);

    fsm.fsm_dbg_pub = nh.advertise<std_msgs::Header>("fsm_dbg", 10);

    fsm.traj_start_trigger_pub = nh.advertise<geometry_msgs::PoseStamped>("traj_start_trigger", 10);

    fsm.stateVisualizer.led_pub = nh.advertise<visualization_msgs::Marker>("state_led", 10);

    // essential for publishers and subscribers to get ready
    ros::Duration(0.5).sleep();

    // 等待遥控器指令
    if (skip_wait_for_rc) {
        ROS_INFO("[N3CTRL] Simulation, skip rc.");
    } else {
        ROS_INFO("[N3CTRL] Waiting for rc");
        while (ros::ok()) {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now())) {
                ROS_INFO("[N3CTRL] rc received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }

    // ros::Timer timer = nh.createTimer(ros::Duration(1.0/1000.0),
    // 	boost::bind(&N3CtrlFSM::process, &fsm, _1));

    // 此处2000Hz不做准，process重新定频率了
    ros::Rate r(2000.0);
    fsm.last_ctrl_time = ros::Time::now();
    // ---- process ----
    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();
        // 循环执行此句
        fsm.process();
    }

    return 0;
}
