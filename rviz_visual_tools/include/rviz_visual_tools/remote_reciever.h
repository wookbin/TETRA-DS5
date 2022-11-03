/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik Consulting
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
 *   * Neither the name of PickNik Consulting nor the names of its
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
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Object for wrapping remote control functionality
*/

#ifndef RVIZ_VISUAL_TOOLS_REMOTE_RECIEVER_H
#define RVIZ_VISUAL_TOOLS_REMOTE_RECIEVER_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>


namespace rviz_visual_tools
{
class RemoteReciever
{
  std_msgs::String str_msg;
  std_msgs::String str_btn_msg;

public:
  RemoteReciever()
  {
    gui_btn_publisher_ = nh_.advertise<std_msgs::String>("/rviz_visual_tools_gui_btn", 1);
    location_name_publisher_ = nh_.advertise<std_msgs::String>("/rviz_visual_tools_gui_location_name", 1);
    goto_location_name_publisher_ = nh_.advertise<std_msgs::String>("/rviz_visual_tools_gui_goto_location_name", 1);
  }

  void publishHOME()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "HOME (not implemented yet)");
    str_btn_msg.data = "HOME";
    goto_location_name_publisher_.publish(str_btn_msg);
    
  }

  void publishGoto(std::string strLocation_name)
  {
    ROS_DEBUG_STREAM_NAMED("gui", "Goto (not implemented yet)");
    str_btn_msg.data = strLocation_name;
    goto_location_name_publisher_.publish(str_btn_msg);
    
  }

  void publishStop()
  {
    ROS_DEBUG_STREAM_NAMED("gui", "Stop (not implemented yet)");
    str_btn_msg.data = "STOP";
    gui_btn_publisher_.publish(str_btn_msg);
    
  }

  //add...////////////////////////////////////////////////////////////////////////////////
  void publishSetLocation(std::string strLocation_name)
  {
    ROS_DEBUG_STREAM_NAMED("gui", "Location_save (not implemented yet)");

    str_msg.data = strLocation_name;
    location_name_publisher_.publish(str_msg);

  }


  ////////////////////////////////////////////////////////////////////////////////////////
  
protected:
  // The ROS publishers
  ros::Publisher gui_btn_publisher_;
  ros::Publisher location_name_publisher_;
  ros::Publisher goto_location_name_publisher_;
  // The ROS node handle.
  ros::NodeHandle nh_;
};

}  // end namespace rviz_visual_tools

#endif  // RVIZ_VISUAL_TOOLS_REMOTE_RECIEVER_H
