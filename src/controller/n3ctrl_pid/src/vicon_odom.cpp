//头文件
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;

bool init_ok = false;
string object_name;

Eigen::Vector3d pos_now, pos_last;
Eigen::Vector3d vel_0, vel_1, vel_2, vel_3, vel_4, vel_filter;
nav_msgs::Odometry Drone_odom;
ros::Time now_t, last_odom_t, last_path_t;


//---------------------------------------vicon定位相关------------------------------------------
Eigen::Vector3d pos_drone_mocap; //无人机当前位置 (vicon)
Eigen::Quaterniond q_mocap;
Eigen::Vector3d Euler_mocap; //无人机当前姿态 (vicon)

ros::Publisher odom_pub;

void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if ( !init_ok )
    {
        init_ok     = true;

        pos_now << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        pos_last = pos_now;
        vel_0 << 0.0, 0.0, 0.0;
        vel_1 << 0.0, 0.0, 0.0;
        vel_2 << 0.0, 0.0, 0.0;
        vel_3 << 0.0, 0.0, 0.0;
        vel_4 << 0.0, 0.0, 0.0;
        vel_filter << 0.0, 0.0, 0.0;

        last_odom_t = msg->header.stamp;
    }
    else
    {
        now_t = msg->header.stamp;

        
        pos_now.x() = msg->pose.position.x;
        pos_now.y() = msg->pose.position.y;
        pos_now.z() = msg->pose.position.z;
        // now_Quat.w() = msg->pose.orientation.w;
        // now_Quat.x() = msg->pose.orientation.x;
        // now_Quat.y() = msg->pose.orientation.y;
        // now_Quat.z() = msg->pose.orientation.z;

        // Q_w = now_Quat.normalized( ).toRotationMatrix( );

        /** velocity filter **/
        vel_0 = ( pos_now - pos_last ) / ( now_t - last_odom_t ).toSec( );        
        vel_filter = ( vel_0 + vel_1 + vel_2 + vel_3 + vel_4 ) * 0.2;
        vel_4 = vel_3;
        vel_3 = vel_2;
        vel_2 = vel_1;
        vel_1 = vel_0;
        //        std::cout << " time " << ( now_t - last_t ).toSec( ) << std::endl;
        //        std::cout << " vel_0 " << vel_0 << std::endl;

        Drone_odom.header.stamp            = now_t;
        Drone_odom.header.frame_id         = "world";
        Drone_odom.child_frame_id          = "world";
        Drone_odom.pose.pose.position.x    = pos_now.x();
        Drone_odom.pose.pose.position.y    = pos_now.y();
        Drone_odom.pose.pose.position.z    = pos_now.z();
        Drone_odom.pose.pose.orientation = msg->pose.orientation;
        Drone_odom.twist.twist.linear.x    = vel_filter.x(); // now_vel.x();
        Drone_odom.twist.twist.linear.y    = vel_filter.y(); // now_vel.y();
        Drone_odom.twist.twist.linear.z    = vel_filter.z(); // now_vel.z();
        odom_pub.publish(Drone_odom);

        last_odom_t = now_t;
        pos_last = pos_now;
    }

}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vicon_odom");
    ros::NodeHandle nh("~");

    nh.param<string>("object_name", object_name, "dji_n3");

    ros::Subscriber mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/"+ object_name + "/pose", 100, mocap_cb);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/dji_n3/Drone_odom", 10);

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

