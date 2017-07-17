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

#ifndef int_markers_to_pose_array_THREAD_H_
#define int_markers_to_pose_array_THREAD_H_

// ROS headers
#include <ros/ros.h>

// TF headers
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// GEOMETRY_MSGS headers
#include <geometry_msgs/PoseArray.h>

// Int Markers headers
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/interactive_marker_client.h>

class IntMarkersToPoseArray
{
  typedef visualization_msgs::InteractiveMarkerInitConstPtr InitConstPtr;
  typedef visualization_msgs::InteractiveMarkerUpdateConstPtr UpdateConstPtr;
  typedef visualization_msgs::InteractiveMarkerUpdatePtr UpdatePtr;

  /** \brief Subscribes to the Interactive Markers and Publishes a Pose Array.
    * 
    * \author Dimitrios Kanoulas
    */
  public:      
    /** \brief Default contructor. */
    IntMarkersToPoseArray (int argc=0, char **argv=NULL);
    
    /** \brief Init the values of the parameters. */
    void
    initParams ();
    
    /** \brief Update the values of the parameters. */
    void
    updateParams (ros::NodeHandle &nh);
    
    /** \brief Update the values of the options. */
    void
    updateOpts (ros::NodeHandle &nh);
    
    /** \brief Publish the Pose Array. */
    void
    publishPoseArray (const ros::TimerEvent&);
          
    /** \brief Whether to publish the set of foothold affordances. */
    bool publish_footholds_;
    
  protected:
    /** \brief TBD */
    void
    initCb (const InitConstPtr& init_msg);
    
    /** \brief TBD */
    void
    updateCb (const UpdateConstPtr& up_msg);
    
    /** \brief TBD */
    void
    statusCb(interactive_markers::InteractiveMarkerClient::StatusT status,
              const std::string& server_id,
              const std::string& status_text);
    
    /** \brief TBD */
    void
    resetCb(const std::string& server_id);

    /** \brief Republish various data topics in the loop. */
    void
    repubData ();
    
    /** \brief Reset the patch messages to int marker poses. */
    void
    resetPatchMsg ();
    
    ros::Timer publish_timer;
    
    //geometry_msgs::PoseArray patch_frames_msg_;
    geometry_msgs::Pose patch_pose_msg_;
    
    /** \brief Foothold patch affordances. */
    geometry_msgs::PoseStamped pp_msg_;
    
    /** \brief Exported foothold patch affordances vector. */
    geometry_msgs::PoseArray pp_msg_vec_;
    
  private:
    /** \brief ROS node handler. */
    ros::NodeHandle nh;
    
    /** \brief The goal frame, i.e. the frame that the patch need to be
      * converted for the traj planner and controller.
      */
    std::string GOAL_FRAME;
    
    /** \brief The range sensor frame and topic. */
    std::string RANGE_SENSOR_FRAME, RANGE_SENSOR_TOPIC;
    
    /** \brief Publishers. */
    ros::Publisher patch_pose_pub_;
    
    /** \brief Interactive markers server. */
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> int_server_;
    
    /** \brief Interactive markers client. */
    boost::shared_ptr<interactive_markers::InteractiveMarkerClient> im_client_;
    
    /** \brief Interactive markers namespace. */
    std::string topic_ns_;
    
    /** \brief Interactive markers frame. */
    std::string target_frame_;
          
    /** \brief Transform poses. */
    tf::TransformListener tf_, listener;
    
    int num_markers_;
};
#endif