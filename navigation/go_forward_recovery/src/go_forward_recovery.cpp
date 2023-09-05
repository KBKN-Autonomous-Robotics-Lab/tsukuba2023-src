/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Copyright (c) 2017, Ryo Okazaki.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*         Ryo Okazaki
*********************************************************************/
#include <go_forward_recovery/go_forward_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <sensor_msgs/LaserScan.h>

//register this planner as a RecoveryBehavior plugin

PLUGINLIB_EXPORT_CLASS(go_forward_recovery::GoForwardRecovery, nav_core::RecoveryBehavior)

namespace go_forward_recovery {
GoForwardRecovery::GoForwardRecovery(): local_costmap_(NULL), 
  initialized_(false), world_model_(NULL) {} 

ros::NodeHandle n;
ros::Publisher vel_pub;
ros::Subscriber scan_sub;
geometry_msgs::Twist cmd_vel;


void GoForwardRecovery::initialize(std::string name, tf2_ros::Buffer*,
                                costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    local_costmap_ = local_costmap;
    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());
    initialized_ = true;
  }else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

GoForwardRecovery::~GoForwardRecovery(){
  delete world_model_;
}

/*
double GoForwardRecovery::null_check(double target){
  if(!(target > 0)){
    target = (double)RANGE_MAX;
    //ROS_WARN("RANGE OVER");
  }
  return target;
}
*/

void GoForwardRecovery::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  scan_sub = n.subscribe("/scan", 10, &GoForwardRecovery::scanCallback, this);
  
  double center_number = (-msg->angle_min)/msg->angle_increment;
  double center = msg->ranges[center_number];
  double left = msg->ranges[center_number+128];
  double right = msg->ranges[center_number-128];
  double back_left = msg->ranges[msg->ranges.size()-1];
  
  sub_n = 0;
  sub_flag = 1;
  
  center = null_check(center);
  left = null_check(left);
  right = null_check(right);
  back_left = null_check(back_left);

  ROS_INFO("center: [%lf], left: [%lf], right: [%lf]", center, left, right);
  ROS_INFO("center number: [%lf]", (-msg->angle_min)/msg->angle_increment);

  if(center < 0.5){
    ROS_WARN("center warning!!");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = -1.0;
  }
  if(left < 0.4){
    ROS_WARN("left warning!!");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = -1.0;
  }
  if(right < 0.4){
    ROS_WARN("right warning!!");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 1.0;
  }
  if(center >=0.5 && left >= 0.4 && left < 1.0 && right >= 0.4){
    cmd_vel.linear.x = 0.2;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
  }
  if(center >=0.5 && left >= 1.0 && right >= 0.4 && back_left < 0.35){
    cmd_vel.linear.x = 0.2;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 1.0;
  }
  if(center >=0.5 && left >= 1.0 && right >= 0.4 && back_left >= 0.35){
    cmd_vel.linear.x = 0.2;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
  }

  ROS_INFO("x: %lf, y: %lf, z: %lf", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
  
  vel_pub.publish(cmd_vel);
  ROS_INFO("sub_n: %d", sub_n);
  sub_n++;
  if(sub_n > 100){
    scan_sub.shutdown();
    sub_flag = 0;
    return;
  }
}

void GoForwardRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if (local_costmap_ == NULL)
  {
    ROS_ERROR("The costmap passed to the RotateRecovery object cannot be NULL. Doing nothing.");
    return;
  }

  ROS_WARN("!!!!!Go forward recovery behavior started.!!!!!");

  while(n.ok()){
    if(sub_flag == 0){
      return;
    }
  }
}
};
