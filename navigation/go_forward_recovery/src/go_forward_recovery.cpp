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

void GoForwardRecovery::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
  vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  scan_sub = n.subscribe("/scan", 10, &GoForwardRecovery::scanCallback, this);
  
  std::vector<float> ranges = scan_msg->ranges;
  
  sub_n = 0;
  sub_flag = 1;
  
  //center
  const int center_start_index = 250;
  const int center_end_index = 350;

  float centerRange = ranges[250];

  for (int i = center_start_index; i <= center_end_index; ++i){
      if (ranges[i] < centerRange){
              centerRange = ranges[i];
      }
  }

  //ROS_INFO("center miniranges %f", centerRange);
    
  //right
  const int right_start_index = 400;
  const int right_end_index = 500;
  
  float rightRange = ranges[400];

  for (int i = right_start_index; i <= right_end_index; ++i){
      if (ranges[i] < rightRange){
            rightRange = ranges[i];
      }
    }
  
  //ROS_INFO("right miniranges %f", rightRange);
    
  //left
  const int left_start_index = 100;
  const int left_end_index = 200;

  float leftRange = ranges[100];

  for (int i = left_start_index; i <= left_end_index; ++i){
      if (ranges[i] < leftRange){
            leftRange = ranges[i];
      }
  }

  //ROS_INFO("left miniranges %f", leftRange);
    
  if (centerRange < 1 && sub_n == 0){
      ROS_WARN("center warn");
      cmd_vel.angular.z = -0.5;  // Set right rotation speed
      ROS_INFO("Start of right rotation");
      vel_pub.publish(cmd_vel);
      sleep(2);  // Time of right rotation
      // stop
      cmd_vel.angular.z = 0.0;
      ROS_INFO("Right rotation stop");
      vel_pub.publish(cmd_vel);
      sleep(2); 
  }
  else if (rightRange < 1 && sub_n == 0){
      ROS_WARN("right warn");            
      cmd_vel.angular.z = 0.5;  // Set right rotation speed
      ROS_INFO("Start of right rotation");
      vel_pub.publish(cmd_vel);
      sleep(2);  // Time of right rotation
      // stop
      cmd_vel.angular.z = 0.0;
      ROS_INFO("Right rotation stop");
      vel_pub.publish(cmd_vel);
      sleep(2); 
  }
  else if (leftRange < 1 && sub_n == 0){
      ROS_WARN("left warn");
      cmd_vel.angular.z = -0.5;  // Set right rotation speed
      ROS_INFO("Start of right rotation");
      vel_pub.publish(cmd_vel);
      sleep(2);  // Time of right rotation
      // stop
      cmd_vel.angular.z = 0.0;
      ROS_INFO("Right rotation stop");
      vel_pub.publish(cmd_vel);
      sleep(2); 
  }
    //ROS_INFO("x: %lf, y: %lf, z: %lf", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
  
  ROS_INFO("sub_n: %d", sub_n);
  sub_n++;
  if(sub_n > 10){ //100
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
    sleep(1);
  }
}
};
