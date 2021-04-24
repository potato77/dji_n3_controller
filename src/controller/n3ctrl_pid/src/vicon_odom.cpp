//头文件
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;

float yaw_offset;
string object_name;

//---------------------------------------vicon定位相关------------------------------------------
Eigen::Vector3d pos_drone_mocap; //无人机当前位置 (vicon)
Eigen::Quaterniond q_mocap;
Eigen::Vector3d Euler_mocap; //无人机当前姿态 (vicon)

ros::Publisher odom_pub;

void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // Read the Drone Position from the Vrpn Package [Frame: Vicon]  (Vicon to ENU frame)
    pos_drone_mocap = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    pos_drone_mocap[0] = pos_drone_mocap[0];
    pos_drone_mocap[1] = pos_drone_mocap[1];
    pos_drone_mocap[2] = pos_drone_mocap[2];
    // Read the Quaternion from the Vrpn Package [Frame: Vicon[ENU]]
    q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

    // Transform the Quaternion to Euler Angles
    // Euler_mocap = quaternion_to_euler(q_mocap);
    
    nav_msgs::Odometry Drone_odom;

    Drone_odom.header.stamp = ros::Time::now();
    Drone_odom.header.frame_id = "world";
    Drone_odom.child_frame_id = "world";

    Drone_odom.pose.pose.position.x = pos_drone_mocap[0];
    Drone_odom.pose.pose.position.y = pos_drone_mocap[1];
    Drone_odom.pose.pose.position.z = pos_drone_mocap[2];

    Drone_odom.pose.pose.orientation = msg->pose.orientation;
    Drone_odom.twist.twist.linear.x = 0.0;
    Drone_odom.twist.twist.linear.y = 0.0;
    Drone_odom.twist.twist.linear.z = 0.0;
    odom_pub.publish(Drone_odom);
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vicon_odom");
    ros::NodeHandle nh("~");

    nh.param<string>("object_name", object_name, "dji_n3");

    ros::Subscriber mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/"+ object_name + "/pose", 100, mocap_cb);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/dji_n3/odom", 10);

    // 频率
    ros::Rate rate(100);

    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while (ros::ok())
    {
        //回调一次 更新传感器状态
        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}
