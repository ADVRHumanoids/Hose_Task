/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017-, Dimitrios Kanoulas
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */

#include <int_markers_to_pose_array.h>

////////////////////////////////////////////////////////////////////////////////
IntMarkersToPoseArray::IntMarkersToPoseArray (int argc, char **argv)
{
  // Initialize the parameters
  initParams ();
  
  // Update the parameters
  updateParams (nh);
  
  // Publish rate 
  double publish_rate = 30;
      
  // subscribers
  topic_ns_ = nh.resolveName("extinguisher_client");
    
  // publishers
  pose_stambed_pub_ = nh.advertise<geometry_msgs::PoseStamped>("hose_grasp_pose", 1);
    
  markers_sub_ = nh.subscribe<visualization_msgs::Marker>
    (topic_ns_, 10, boost::bind(&IntMarkersToPoseArray::markerCB, this, _1));
}

////////////////////////////////////////////////////////////////////////////////
void
IntMarkersToPoseArray::initParams ()
{
  this->GOAL_FRAME = "/base_link";
  this->RANGE_SENSOR_FRAME = "/base_link";
  this->publish_pose_ = false;
}

////////////////////////////////////////////////////////////////////////////////
void
IntMarkersToPoseArray::updateParams (ros::NodeHandle &nh)
{
  nh.getParam("/Hose_Task/GOAL_FRAME", this->GOAL_FRAME);
  nh.getParam("/Hose_Task/RANGE_SENSOR_FRAME", this->RANGE_SENSOR_FRAME);
  nh.getParam("/Hose_Task/publish_pose_", this->publish_pose_);
}

void
IntMarkersToPoseArray::markerCB (const visualization_msgs::Marker::ConstPtr & msg)
{
  std::cout << "markerCB" << std::endl;
  if (publish_pose_)
  {
    pp_msg_.header.frame_id = this->GOAL_FRAME;
    pp_msg_.header.stamp = ros::Time(0);
    pp_msg_.pose.orientation = msg->pose.orientation;
    pp_msg_.pose.position = msg->pose.position;

    pose_stambed_pub_.publish (pp_msg_);
  }
}