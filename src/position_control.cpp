/***************************************************************************
* Copyright (C) 2013 - 2014 by *
* Tarek Taha, Khalifa University Robotics Institute KURI *
* <tarek.taha@kustar.ac.ae> *
* *
* *
* This program is free software; you can redistribute it and/or modify *
* it under the terms of the GNU General Public License as published by *
* the Free Software Foundation; either version 2 of the License, or *
* (at your option) any later version. *
* *
* This program is distributed in the hope that it will be useful, *
* but WITHOUT ANY WARRANTY; without even the implied warranty of *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the *
* GNU General Public License for more details. *
* *
* You should have received a copy of the GNU General Public License *
* along with this program; if not, write to the *
* Free Software Foundation, Inc., *
* 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA. *
***************************************************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include <Eigen/Eigen>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_position_controller");
    ros::NodeHandle n;
    ros::Publisher uavPosePublisher = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint/local_position", 1000);
    ros::Rate loop_rate(50);
    geometry_msgs::PoseStamped newPose;
    double x = 0.5;
    double y = 0.5;
    double z = 1.0;
    
    while (ros::ok())
    {
        newPose.pose.position.x  = x;
        newPose.pose.position.y  = y;
        newPose.pose.position.z  = z;
        
        newPose.header.stamp=ros::Time::now();
        newPose.header.frame_id="world";       
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,0);
        newPose.pose.orientation.w = quaternion.w();
        newPose.pose.orientation.x = quaternion.x();
        newPose.pose.orientation.y = quaternion.y();
        newPose.pose.orientation.z = quaternion.z();
        
        uavPosePublisher.publish(newPose);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
