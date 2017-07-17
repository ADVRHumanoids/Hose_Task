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
  target_frame_ = nh.resolveName(RANGE_SENSOR_FRAME);
  topic_ns_ = nh.resolveName("extinguisher_server");
  
  int_server_.reset (new interactive_markers::InteractiveMarkerServer ("new_foot_marker","",false));
  
  im_client_.reset (new interactive_markers::InteractiveMarkerClient(tf_,
                                                                      target_frame_,
                                                                      topic_ns_));
  im_client_->setInitCb(boost::bind(&IntMarkersToPoseArray::initCb, this, _1));
  im_client_->setUpdateCb(boost::bind(&IntMarkersToPoseArray::updateCb, this, _1));
  im_client_->setResetCb(boost::bind(&IntMarkersToPoseArray::resetCb, this, _1));
  im_client_->setStatusCb(boost::bind(&IntMarkersToPoseArray::statusCb, this, _1, _2, _3));
  im_client_->subscribe(topic_ns_);
  
  // publishers
  patch_pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("hose_grasp_pose", 1);
  
  publish_timer = nh.createTimer(ros::Duration(1/publish_rate),
                                 &IntMarkersToPoseArray::publishPoseArray, this);
}

////////////////////////////////////////////////////////////////////////////////
void
IntMarkersToPoseArray::initParams ()
{
  this->GOAL_FRAME = "/camera_rgb_optical_frame";
  this->RANGE_SENSOR_FRAME = "/camera_rgb_optical_frame";
  this->RANGE_SENSOR_TOPIC = "/camera/depth_registered/points";
      
  // options
  publish_footholds_ = false;
}

////////////////////////////////////////////////////////////////////////////////
void
IntMarkersToPoseArray::updateParams (ros::NodeHandle &nh)
{
  nh.getParam("/footstep_patch/GOAL_FRAME", this->GOAL_FRAME);
  nh.getParam("/footstep_patch/RANGE_SENSOR_FRAME", this->RANGE_SENSOR_FRAME);
  nh.getParam("/footstep_patch/RANGE_SENSOR_TOPIC", this->RANGE_SENSOR_TOPIC);
}

////////////////////////////////////////////////////////////////////////////////
void
IntMarkersToPoseArray::updateOpts (ros::NodeHandle &nh)
{
  nh.getParam("/footstep_patch/publish_footholds_", this->publish_footholds_);
}

////////////////////////////////////////////////////////////////////////////////
void
IntMarkersToPoseArray::initCb (const InitConstPtr& init_msg)
{
  std::cout << "Hey you" << std::endl;
  num_markers_ = init_msg->markers.size();
  
  UpdatePtr update (new visualization_msgs::InteractiveMarkerUpdate ());
  update->markers = init_msg->markers;
  update->seq_num = init_msg->seq_num;
  update->server_id = init_msg->server_id;
      
  visualization_msgs::InteractiveMarker im;
  for (int i=0; i<num_markers_; i++)
    int_server_->insert (update->markers[i]);
}

////////////////////////////////////////////////////////////////////////////////
void
IntMarkersToPoseArray::updateCb (const UpdateConstPtr& up_msg)
{
}

////////////////////////////////////////////////////////////////////////////////
void
IntMarkersToPoseArray::statusCb (interactive_markers::InteractiveMarkerClient::StatusT status,
                                  const std::string& server_id,
                                  const std::string& status_text)
{
}

////////////////////////////////////////////////////////////////////////////////
void
IntMarkersToPoseArray::resetCb(const std::string& server_id)
{
}

////////////////////////////////////////////////////////////////////////////////
void
IntMarkersToPoseArray::repubData ()
{    
  // Republish topics
  if (publish_footholds_)
  {
    std::cout << "Publish footholds" << std::endl;
    resetPatchMsg ();
    patch_pose_pub_.publish (pp_msg_vec_);
    nh.setParam("footstep_patch/publish_footholds_", false);
  }
}

////////////////////////////////////////////////////////////////////////////////
void
IntMarkersToPoseArray::resetPatchMsg ()
{
  pp_msg_vec_.poses.clear();
  visualization_msgs::InteractiveMarker im;
  
  for (int i=0; i<num_markers_; i++)
  {
    std::stringstream ss;
    ss << i;
    int_server_->get("my_marker" + ss.str(),im);
    patch_pose_msg_ = im.pose;
    
    // Create the PoseStamped msg
    pp_msg_.header.frame_id = RANGE_SENSOR_FRAME;
    pp_msg_.header.stamp = ros::Time(0);
    pp_msg_.pose.orientation = patch_pose_msg_.orientation;
    pp_msg_.pose.position = patch_pose_msg_.position;
    
    // transform the patch in the correct frame
    try
    {
      listener.transformPose (GOAL_FRAME, pp_msg_, pp_msg_);
      pp_msg_.header.frame_id = GOAL_FRAME;
      pp_msg_.header.stamp = ros::Time(0);
      patch_pose_msg_.orientation = pp_msg_.pose.orientation;
      patch_pose_msg_.position = pp_msg_.pose.position;
      
      pp_msg_vec_.header.frame_id = pp_msg_.header.frame_id;
      pp_msg_vec_.poses.push_back (patch_pose_msg_);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("No transformation available: %s", ex.what());
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void
IntMarkersToPoseArray::publishPoseArray (const ros::TimerEvent&)
{    
  updateOpts (nh);
  repubData ();
  
  im_client_->update ();
  int_server_->applyChanges();
}

/*
int
main (int argc, char** argv)
{
  ros::init (argc, argv, "int_markers_to_pose_array");
  
  IntMarkersToPoseArray imtpa;
  
  ros::spin ();
  
  return 0;
}*/