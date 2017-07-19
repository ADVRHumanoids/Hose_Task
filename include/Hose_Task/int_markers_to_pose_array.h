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
#include <geometry_msgs/PoseStamped.h>

// Int Markers headers
#include <visualization_msgs/Marker.h>

class IntMarkersToPoseArray
{
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
    
    /** \brief Publish the Pose Array. */
    void
    publishPoseArray (const ros::TimerEvent&);
        
    ros::Subscriber markers_sub_;
    
    bool publish_pose_;
    
  protected:
    void
    markerCB (const visualization_msgs::Marker::ConstPtr & msg);
            
    /** \brief Foothold patch affordances. */
    geometry_msgs::PoseStamped pp_msg_;
        
  private:
    /** \brief ROS node handler. */
    ros::NodeHandle nh;
    
    /** \brief The goal frame, i.e. the frame that the patch need to be
      * converted for the traj planner and controller.
      */
    std::string GOAL_FRAME;
    
    /** \brief The range sensor frame and topic. */
    std::string RANGE_SENSOR_FRAME;
    
    /** \brief Publishers. */
    ros::Publisher pose_stambed_pub_;
            
    /** \brief Markers namespace. */
    std::string topic_ns_;
};
#endif