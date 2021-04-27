#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include "sample_waypoints.h"
#include <vector>
#include <deque>
#include <boost/format.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using bfmt = boost::format;

ros::Publisher pub1;
ros::Publisher pub2;
nav_msgs::Path waypoints;

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "waypoint_generator_simple");
    ros::NodeHandle n("~");

    // ego 中订阅了该话题,如果target_type=1
    pub1 = n.advertise<nav_msgs::Path>("/waypoint_generator/waypoints", 50);
    pub2 = n.advertise<geometry_msgs::PoseArray>("waypoints_vis", 10);

    geometry_msgs::PoseStamped pt;
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
        
        pt.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_sp/180*3.1415926);
        
        pt.pose.position.x = pos_sp[0];
        pt.pose.position.y = pos_sp[1];
        pt.pose.position.z = pos_sp[2];
        waypoints.poses.push_back(pt);     

        waypoints.header.frame_id = std::string("world");
        waypoints.header.stamp = ros::Time::now();

        pub1.publish(waypoints);
        waypoints.poses.clear();

        // geometry_msgs::PoseStamped init_pose;
        // init_pose.header = odom.header;
        // init_pose.pose = odom.pose.pose;
        // waypoints.poses.insert(waypoints.poses.begin(), init_pose);
        // 

        ros::spinOnce();
        sleep(1.0); 
    }

    return 0;
}
