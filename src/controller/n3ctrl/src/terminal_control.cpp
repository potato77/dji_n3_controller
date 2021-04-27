#include <ros/ros.h>
#include <iostream>

#include <quadrotor_msgs/PositionCommand.h>

using namespace std;

//即将发布的command
quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};
//发布
ros::Publisher pos_cmd_pub;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>　主函数　<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "terminal_control");
    ros::NodeHandle nh;

    //　【发布】　控制指令
    pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/terminal/point_cmd", 50);
    // /terminal/point_cmd

    /* control parameter */
    cmd.kx[0] = pos_gain[0];
    cmd.kx[1] = pos_gain[1];
    cmd.kx[2] = pos_gain[2];

    cmd.kv[0] = vel_gain[0];
    cmd.kv[1] = vel_gain[1];
    cmd.kv[2] = vel_gain[2];

    cmd.trajectory_id = 1;

    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    //cout.setf(ios::showpos);

    float pos_sp[3];
    float yaw_sp;

    while(ros::ok())
    {
        // Waiting for input
        cout << ">>>>>>>>>>>>>>>> Welcome to use dji Terminal Control <<<<<<<<<<<<<<<<"<< endl;

        cout << "Please input the reference state [x y z yaw]: "<< endl;
        cout << "setpoint_t[0] --- x [m] : "<< endl;
        cin >> pos_sp[0];
        cout << "setpoint_t[1] --- y [m] : "<< endl;
        cin >> pos_sp[1];
        cout << "setpoint_t[2] --- z [m] : "<< endl;
        cin >> pos_sp[2];
        cout << "setpoint_t[3] --- yaw [du] : "<< endl;
        cin >> yaw_sp;


        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "world";
        cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
        cmd.trajectory_id = cmd.trajectory_id + 1;

        cmd.position.x = pos_sp[0];
        cmd.position.y = pos_sp[1];
        cmd.position.z = pos_sp[2];

        cmd.velocity.x = 0.0;
        cmd.velocity.y = 0.0;
        cmd.velocity.z = 0.0;

        cmd.acceleration.x = 0.0;
        cmd.acceleration.y = 0.0;
        cmd.acceleration.z = 0.0;

        cmd.yaw = yaw_sp/180*3.1415926;
        cmd.yaw_dot = 0.0;

        pos_cmd_pub.publish(cmd);

        ros::spinOnce();
        sleep(1.0); 
    }

    return 0;
}
