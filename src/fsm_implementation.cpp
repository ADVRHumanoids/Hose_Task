/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Dimitrios Kanoulas
 * email: dkanoulas@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
 */

#include "fsm_definition.h"
#include <string>
#include <eigen_conversions/eigen_msg.h>

/******************************** BEGIN Home *********************************/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Home::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Home::entry (const XBot::FSM::Message& msg) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Home::run (double time, double period)
{
//  //std::cout << "Home run" << std::endl;
//  
//  // Home the robot
//  localTimer = localTimer >1 ? 1: localTimer;
//  
//  shared_data()._robot->setPositionReference
//    (shared_data()._q0+localTimer*(shared_data().state[0]-shared_data()._q0));
//  shared_data()._robot->move();
//  
//  if (localTimer == 1)
//  {
//    // Homing finished, so move to the ne state
//    shared_data()._robot->getJointPosition(shared_data()._q0);
//    localTimer = 0;
//    
//    // New state transit
    transit ("Move_RH");
//  }
//  
//  localTimer += localStep;
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Home::exit () {}

/*END Home*/


/******************************* BEGIN Move_RH *******************************/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_RH::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_RH::entry (const XBot::FSM::Message& msg)
{
  std::cout << "Move_RH entry" << std::endl;

  // Blocking Reading: wait for a command
  if(!shared_data().command.read(shared_data().current_command))
    std::cout << shared_data().current_command.str() << std::endl;

  // Wait for RH_Pose, i.e. the Hose Grasp Pose (hose_grasp_pose)
  shared_data()._hose_grasp_pose =
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>("hose_grasp_pose");

  // Debug msg
  std::cout << shared_data()._hose_grasp_pose->pose.position.x << std::endl;


  // Move RH to RH_Pose (1 mid point in the z-axis fixed dist)

  // sense and sync model
  shared_data()._robot->sense();
  Eigen::Affine3d world_T_bl;
  std::string fb;
  shared_data()._robot->model().getFloatingBaseLink(fb);
  tf.getTransformTf(fb, "world_odom", world_T_bl);
  shared_data()._robot->model().setFloatingBasePose(world_T_bl);
  shared_data()._robot->model().update();

  // Get current hand
  KDL::Frame hand_pose_KDL;
  Eigen::Affine3d hand_pose;
  geometry_msgs::Pose start_hand_pose;

  // Get hand pose
  shared_data()._robot->model().getPose("LSoftHand", hand_pose);
  shared_data().sl_hand_pose = hand_pose;
  
  // Transform from Eigen::Affine3d to geometry_msgs::Pose
  tf::poseEigenToMsg (hand_pose, start_hand_pose);

  // Define the start frame as geometry_msgs::PoseStamped
  geometry_msgs::PoseStamped start_hand_pose_stamped;
  start_hand_pose_stamped.pose = start_hand_pose;
  
  // Create the Cartesian trajectories
  trajectory_utils::Cartesian start_traj;
  start_traj.distal_frame = "LSoftHand";
  start_traj.frame = start_hand_pose_stamped;
  
  // define the end frame
  geometry_msgs::PoseStamped end_hand_pose_stamped;
  end_hand_pose_stamped.pose = start_hand_pose;
  
  end_hand_pose_stamped.pose.position.x = shared_data()._hose_grasp_pose->pose.position.x;
  end_hand_pose_stamped.pose.position.y = shared_data()._hose_grasp_pose->pose.position.y;
  end_hand_pose_stamped.pose.position.z = shared_data()._hose_grasp_pose->pose.position.z+0.2;
    
  end_hand_pose_stamped.pose.orientation.x = shared_data()._hose_grasp_pose->pose.orientation.x;
  end_hand_pose_stamped.pose.orientation.y = shared_data()._hose_grasp_pose->pose.orientation.y;
  end_hand_pose_stamped.pose.orientation.z = shared_data()._hose_grasp_pose->pose.orientation.z;
  end_hand_pose_stamped.pose.orientation.w = shared_data()._hose_grasp_pose->pose.orientation.w;
  
  trajectory_utils::Cartesian end;
  end.distal_frame = "LSoftHand";
  end.frame = end_hand_pose_stamped;


  // define the first segment
  trajectory_utils::segment s1;
  s1.type.data = 0;        // min jerk traj
  s1.T.data = 10.0;        // traj duration 1 second      
  s1.start = start_traj;   // start pose
  s1.end = end;            // end pose 

  start_traj.frame = end_hand_pose_stamped;
  
  end_hand_pose_stamped.pose.position.z = shared_data()._hose_grasp_pose->pose.position.z;

  
  end.frame = end_hand_pose_stamped;
  
  // define the first segment
  trajectory_utils::segment s2;
  s2.type.data = 0;        // min jerk traj
  s2.T.data = 10.0;        // traj duration 1 second      
  s2.start = start_traj;   // start pose
  s2.end = end;            // end pose 
  
  // only one segment in this example
  std::vector<trajectory_utils::segment> segments;
  segments.push_back (s1);
  segments.push_back (s2);
  
  // prapere the advr_segment_control
  ADVR_ROS::advr_segment_control srv;
  srv.request.segment_trj.header.frame_id = "world_odom";
  srv.request.segment_trj.header.stamp = ros::Time::now();
  srv.request.segment_trj.segments = segments;
  
  // call the service
  shared_data()._client.call(srv);
  
  // save last hand pose
   shared_data()._last_lh_pose =
     boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_hand_pose_stamped));
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_RH::run (double time, double period)
{
  //std::cout << "Move_RH run" << std::endl;
  
  // Magnetise cmd
  
  //std_msgs::Bool msg;
  //msg.data = true;
  //for (int i=0; i<10; i++)
  //  shared_data()._grasp_mag_pub.publish (msg);
  
  //TBD: Check if the RH has reached the hose_grasp_pose 
  
  // blocking reading: wait for a command
  if(shared_data().command.read(shared_data().current_command))
  {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;

    // RH Move failed
    if (!shared_data().current_command.str().compare("rh_move_fail"))
      transit("Move_RH");
    
    // RH Move Succeeded
    if (!shared_data().current_command.str().compare("success"))
      transit("Grasp_RH");
  }
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_RH::exit () {}

/*END Move_RH*/


 /*BEGIN Grasp_RH*/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_RH::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_RH::entry(const XBot::FSM::Message& msg)
{
     //r hand moving
//     int r_hand_id =_robot->getHand()["r_handj"]->getHandId();
//     XBot::Hand::Ptr r_hand =_robot->getHand(r_hand_id);
//     r_hand->grasp(0);
//     double r_state = r_hand->getGraspState();
//     std::cout<<"R hand STATE "<<r_state<<std::endl;

     //l hand moving
//     int l_hand_id =shared_data()._robot->getHand()["l_handj"]->getHandId();
//     XBot::Hand::Ptr l_hand =shared_data()._robot->getHand(l_hand_id);
//     l_hand->grasp(1);
//    
//     shared_data()._robot->move();
//     
//     shared_data()._robot->sense();
//     double l_state = l_hand->getGraspState();
//     std::cout<<"L hand STATE "<<l_state<<std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_RH::run(double time, double period)
{
  std::cout << "Grasp_RH run" << std::endl;
  
  //TBD: check if the move has failed
  bool move_rh_fail = false;
  
  if (move_rh_fail)
    transit("Move_Fail");
  
  //TBD: Grasp with RH
  
  transit("Grasp_RH_Done");
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_RH::exit ()
{
  std::cout << "Grasp_RH run" << std::endl;
}

/*END Grasp_RH*/


 /*BEGIN Grasp_RH_Done*/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_RH_Done::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_RH_Done::entry (const XBot::FSM::Message& msg)
{
  std::cout << "Grasp_RH_Done entry" << std::endl;

  // Move RH to RH_Pose

  // sense and sync model
  shared_data()._robot->sense();
  Eigen::Affine3d world_T_bl;
  std::string fb;
  shared_data()._robot->model().getFloatingBaseLink(fb);
  tf.getTransformTf(fb, "world_odom", world_T_bl);
  shared_data()._robot->model().setFloatingBasePose(world_T_bl);
  shared_data()._robot->model().update();

  // Define the start frame as geometry_msgs::PoseStamped
  geometry_msgs::PoseStamped start_hand_pose_stamped;
  start_hand_pose_stamped = *shared_data()._last_lh_pose;
  
  // Create the Cartesian trajectories
  trajectory_utils::Cartesian start_traj;
  start_traj.distal_frame = "LSoftHand";
  start_traj.frame = start_hand_pose_stamped;
  
  // define the end frame
  geometry_msgs::PoseStamped end_hand_pose_stamped;
  end_hand_pose_stamped.pose = start_hand_pose_stamped.pose;
  end_hand_pose_stamped.pose.position.z += 0.1;
  
  trajectory_utils::Cartesian end;
  end.distal_frame = "LSoftHand";
  end.frame = end_hand_pose_stamped;


  // define the first segment
  trajectory_utils::segment s1;
  s1.type.data = 0;        // min jerk traj
  s1.T.data = 5.0;        // traj duration 1 second      
  s1.start = start_traj;   // start pose
  s1.end = end;            // end pose   
  
  // only one segment in this example
  std::vector<trajectory_utils::segment> segments;
  segments.push_back (s1);
  
  // prapere the advr_segment_control
  ADVR_ROS::advr_segment_control srv;
  srv.request.segment_trj.header.frame_id = "world_odom";
  srv.request.segment_trj.header.stamp = ros::Time::now();
  srv.request.segment_trj.segments = segments;
  
  // call the service
  shared_data()._client.call(srv);
  
  // save last hand pose
  shared_data()._last_lh_pose =
    boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_hand_pose_stamped));

}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_RH_Done::run (double time, double period)
{
  std::cout << "Grasp_RH_Done run" << std::endl;
  
  //TBD: Check if RH_Grasp or the Move_RH has failed

    
  // blocking reading: wait for a command
  if(shared_data().command.read(shared_data().current_command))
  {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;

    // RH Move failed
    //bool grasp_rh_fail = false;
    //bool move_rh_fail = false;
  
    //if (grasp_rh_fail)
    //  transit("Grasp_Fail");
    
    //if (move_rh_fail)
    //  transit("Move_Fail");
    
    // RH Move Succeeded
    if (!shared_data().current_command.str().compare("success"))
      transit("Orient_RH");
  }
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_RH_Done::exit () {}

/*END Grasp_RH_Done*/


 /*BEGIN Orient_RH*/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_RH::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_RH::entry (const XBot::FSM::Message& msg)
{
  std::cout << "Orient_RH entry" << std::endl;

  //Wait for RH_Pose (orientation)
  shared_data()._hose_grasp_pose =
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>("hose_grasp_pose");

  // Move RH to RH_Pose

  // Define the start frame as geometry_msgs::PoseStamped
  geometry_msgs::PoseStamped start_hand_pose_stamped;
  start_hand_pose_stamped = *shared_data()._last_lh_pose;
  
  // Create the Cartesian trajectories
  trajectory_utils::Cartesian start_traj;
  start_traj.distal_frame = "LSoftHand";
  start_traj.frame = start_hand_pose_stamped;
  
  // define the end frame
  geometry_msgs::PoseStamped end_hand_pose_stamped;
  end_hand_pose_stamped.pose = start_hand_pose_stamped.pose;
      
  end_hand_pose_stamped.pose.orientation.x = shared_data()._hose_grasp_pose->pose.orientation.x;
  end_hand_pose_stamped.pose.orientation.y = shared_data()._hose_grasp_pose->pose.orientation.y;
  end_hand_pose_stamped.pose.orientation.z = shared_data()._hose_grasp_pose->pose.orientation.z;
  end_hand_pose_stamped.pose.orientation.w = shared_data()._hose_grasp_pose->pose.orientation.w;
  
  trajectory_utils::Cartesian end;
  end.distal_frame = "LSoftHand";
  end.frame = end_hand_pose_stamped;


  // define the first segment
  trajectory_utils::segment s1;
  s1.type.data = 0;        // min jerk traj
  s1.T.data = 10.0;        // traj duration 1 second      
  s1.start = start_traj;   // start pose
  s1.end = end;            // end pose   
  
  // only one segment in this example
  std::vector<trajectory_utils::segment> segments;
  segments.push_back (s1);
  
  // prapere the advr_segment_control
  ADVR_ROS::advr_segment_control srv;
  srv.request.segment_trj.header.frame_id = "world_odom";
  srv.request.segment_trj.header.stamp = ros::Time::now();
  srv.request.segment_trj.segments = segments;
  
  // call the service
  shared_data()._client.call(srv);
  
  // save last hand pose
  shared_data()._last_lh_pose =
    boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(end_hand_pose_stamped));

}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_RH::run(double time, double period)
{
  std::cout << "Orient_RH run" << std::endl;
  
  //TBD: Move RH to RH_Pose
  // blocking reading: wait for a command
  if(shared_data().command.read(shared_data().current_command))
  {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;
    
    // RH Move Succeeded
    if (!shared_data().current_command.str().compare("success"))
      transit("Orient_RH_Done");
  }
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_RH::exit () {}

/*END Orient_RH*/


 /*BEGIN Orient_RH_Done*/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_RH_Done::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_RH_Done::entry (const XBot::FSM::Message& msg) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_RH_Done::run (double time, double period)
{
  std::cout << "Orient_RH_Done run" << std::endl;

  //TBD: Check if the RH orientation has failed
  bool orient_fail = false;
  
  if (orient_fail)
    transit("Orient_Fail");
  
  //TBD: Wait LH_Pose
  shared_data()._hose_grasp_pose =
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>("hose_grasp_pose");
    //ros::topic::waitForMessage<geometry_msgs::PoseStamped>("hose_push_pose");
  
  transit("Move_LH");
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_RH_Done::exit () {}

/*END Orient_RH_Done*/


 /*BEGIN Move_LH*/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_LH::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_LH::entry (const XBot::FSM::Message& msg)
{
  // sense and sync model
  shared_data()._robot->sense();
  Eigen::Affine3d world_T_bl;
  std::string fb;
  shared_data()._robot->model().getFloatingBaseLink(fb);
  tf.getTransformTf(fb, "world_odom", world_T_bl);
  shared_data()._robot->model().setFloatingBasePose(world_T_bl);
  shared_data()._robot->model().update();

  // Get currnt hand 
  Eigen::Affine3d hand_pose;
  geometry_msgs::Pose start_frame_pose;
  
  shared_data()._robot->model().getPose("RSoftHand", "world_odom", hand_pose);
  //shared_data()._robot->model().getPose("RSoftHand", hand_pose);
  shared_data().sr_hand_pose = hand_pose;

  tf::poseEigenToMsg (hand_pose, start_frame_pose);
  
  //tf::Quaternion q (start_frame_pose.orientation.x,
  //                  start_frame_pose.orientation.y,
  //                  start_frame_pose.orientation.z,
  //                  start_frame_pose.orientation.w);
  //tf::Quaternion q_inv = q.inverse();
  
  // define the start frame 
  geometry_msgs::PoseStamped start_frame;
  start_frame.pose = start_frame_pose;
  
  // define the end frame
  geometry_msgs::PoseStamped end_frame;
  end_frame.pose = start_frame_pose;
  
  end_frame.pose.position.x = shared_data()._hose_grasp_pose->pose.position.x;
  end_frame.pose.position.y = shared_data()._hose_grasp_pose->pose.position.y;
  end_frame.pose.position.z = shared_data()._hose_grasp_pose->pose.position.z;
  
  end_frame.pose.orientation.x = 0;
  end_frame.pose.orientation.y = -0.7071;
  end_frame.pose.orientation.z = 0;
  end_frame.pose.orientation.w = 0.7071;
  
  trajectory_utils::Cartesian start;
  start.distal_frame = "RSoftHand";
  start.frame = start_frame;
  
  trajectory_utils::Cartesian end;
  end.distal_frame = "RSoftHand";
  end.frame = end_frame;
  
  // define the first segment
  trajectory_utils::segment s1;
  s1.type.data = 0;        // min jerk traj
  s1.T.data = 5.0;         // traj duration 5 second      
  s1.start = start;        // start pose
  s1.end = end;            // end pose 
  
  // only one segment in this example
  std::vector<trajectory_utils::segment> segments;
  segments.push_back (s1);
  
  // prapere the advr_segment_control
  ADVR_ROS::advr_segment_control srv;
  srv.request.segment_trj.header.frame_id = "world_odom";
  //srv.request.segment_trj.header.frame_id = "world";
  srv.request.segment_trj.header.stamp = ros::Time::now();
  srv.request.segment_trj.segments = segments;
  
  // call the service
  shared_data()._client.call(srv);
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_LH::run (double time, double period)
{
  std::cout << "Move_LH run" << std::endl;
  
  //TBD: Grasp with LH
  //TBD: Move to LH_Pose
  
  // blocking reading: wait for a command
  if(shared_data().command.read(shared_data().current_command))
  {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;

    // LH Move Succeeded
    if (!shared_data().current_command.str().compare("success"))
      transit("Push_LH");
  }
  }

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_LH::exit () {}

/*END Move_LH*/


 /*BEGIN Push_LH*/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_LH::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_LH::entry (const XBot::FSM::Message& msg) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_LH::run (double time, double period)
{
  std::cout << "Push_LH run" << std::endl;

  // sense and sync model
  shared_data()._robot->sense();
  Eigen::Affine3d world_T_bl;
  std::string fb;
  shared_data()._robot->model().getFloatingBaseLink(fb);
  tf.getTransformTf(fb, "world_odom", world_T_bl);
  shared_data()._robot->model().setFloatingBasePose(world_T_bl);
  shared_data()._robot->model().update();

  // Get currnt hand 
  Eigen::Affine3d pose;
  geometry_msgs::Pose start_frame_pose;
  
  shared_data()._robot->model().getPose("RSoftHand", pose);
  tf::poseEigenToMsg (pose, start_frame_pose);
    
  // define the start frame 
  geometry_msgs::PoseStamped start_frame;
  start_frame.pose = start_frame_pose;
  
  // define the end frame
  geometry_msgs::PoseStamped end_frame;
  end_frame.pose = start_frame_pose;
 
  trajectory_utils::Cartesian start;
  start.distal_frame = "RSoftHand";
  
  trajectory_utils::Cartesian end;
  end.distal_frame = "RSoftHand";
  
  // define the first segment
  trajectory_utils::segment s1;
  
  
  // only one segment in this example
  std::vector<trajectory_utils::segment> segments;
  
  for (int i=0; i<5; i++)
  {
    start_frame.pose.position.z = end_frame.pose.position.z;
    start.frame = start_frame;

    std::cout << "S end_frame.pose.position.z: " << end_frame.pose.position.z << std::endl;
    end_frame.pose.position.z -= 0.05*pow(-1,i);
    std::cout << "F end_frame.pose.position.z: " << end_frame.pose.position.z << std::endl;
    end.frame = end_frame;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = 2;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = end;            // end pose 
    segments.push_back (s1);
  }
  
  // prapere the advr_segment_control
  ADVR_ROS::advr_segment_control srv;
  srv.request.segment_trj.header.frame_id = "world_odom";
  //srv.request.segment_trj.header.frame_id = "world";
  srv.request.segment_trj.header.stamp = ros::Time::now();
  srv.request.segment_trj.segments = segments;
  
  // call the service
  shared_data()._client.call(srv);

  //TBD: check if the orientation or mnove has failed
  bool orient_fail = false;
  bool push_fail = false;
  
  if (push_fail)
    transit("Push_Fail");
  else if (orient_fail)
    transit("Orient_Fail");
  
  //TBD: Move LH to fixed position a number of times
  
  transit("Push_LH_Done");
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_LH::exit () {}

/*END Push_LH*/


 /*BEGIN Push_LH_Done*/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_LH_Done::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_LH_Done::entry (const XBot::FSM::Message& msg) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_LH_Done::run (double time, double period)
{
  std::cout << "Push_LH_Done run" << std::endl;
  
  //TBD: Ungrasp both hands
  
  transit("Homing");
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_LH_Done::exit () {}

/*END Push_LH_Done*/


 /*BEGIN Homing*/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Homing::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Homing::entry (const XBot::FSM::Message& msg)
{
  std::cout << "Homing entry" << std::endl;

  // Ungrasp left and right hands

  // Home both hands
  // sense and sync model
  shared_data()._robot->sense();
  Eigen::Affine3d world_T_bl;
  std::string fb;
  shared_data()._robot->model().getFloatingBaseLink(fb);
  tf.getTransformTf(fb, "world_odom", world_T_bl);
  shared_data()._robot->model().setFloatingBasePose(world_T_bl);
  shared_data()._robot->model().update();

  // Get current hand
  KDL::Frame hand_pose_KDL;
  Eigen::Affine3d hand_pose;
  geometry_msgs::Pose start_hand_pose;

  // Get hand pose
  shared_data()._robot->model().getPose("LSoftHand", hand_pose);
  
  // Transform from Eigen::Affine3d to geometry_msgs::Pose
  tf::poseEigenToMsg (hand_pose, start_hand_pose);

  // Define the start frame as geometry_msgs::PoseStamped
  geometry_msgs::PoseStamped start_hand_pose_stamped;
  start_hand_pose_stamped.pose = start_hand_pose;
  
  // Create the Cartesian trajectories
  trajectory_utils::Cartesian start_traj;
  start_traj.distal_frame = "LSoftHand";
  start_traj.frame = start_hand_pose_stamped;
  
  // define the end frame
  geometry_msgs::PoseStamped end_hand_pose_stamped;
  tf::poseEigenToMsg (shared_data().sl_hand_pose, end_hand_pose_stamped.pose);

  
  trajectory_utils::Cartesian end;
  end.distal_frame = "LSoftHand";
  end.frame = end_hand_pose_stamped;


  // define the first segment
  trajectory_utils::segment s1;
  s1.type.data = 0;        // min jerk traj
  s1.T.data = 5.0;        // traj duration 1 second      
  s1.start = start_traj;   // start pose
  s1.end = end;            // end pose
  
  // only one segment in this example
  std::vector<trajectory_utils::segment> segments;
  segments.push_back (s1);
  
  // prapere the advr_segment_control
  ADVR_ROS::advr_segment_control srv;
  srv.request.segment_trj.header.frame_id = "world_odom";
  srv.request.segment_trj.header.stamp = ros::Time::now();
  srv.request.segment_trj.segments = segments;
  
  // call the service
  shared_data()._client.call(srv);
  
  if(shared_data().command.read(shared_data().current_command))
  {
    std::cout << "Command: " << shared_data().current_command.str() << std::endl;
  
  if (!shared_data().current_command.str().compare("success"))
  {
  
  // Home right hand
  shared_data()._robot->sense();
  Eigen::Affine3d world_T_bl;
  std::string fb;
  shared_data()._robot->model().getFloatingBaseLink(fb);
  tf.getTransformTf(fb, "world_odom", world_T_bl);
  shared_data()._robot->model().setFloatingBasePose(world_T_bl);
  shared_data()._robot->model().update();

  // Get current hand
  KDL::Frame hand_pose_KDL_r;
  Eigen::Affine3d hand_pose_r;
  geometry_msgs::Pose start_hand_pose_r;

  // Get hand pose
  shared_data()._robot->model().getPose("RSoftHand", hand_pose_r);
  
  // Transform from Eigen::Affine3d to geometry_msgs::Pose
  tf::poseEigenToMsg (hand_pose_r, start_hand_pose_r);

  // Define the start frame as geometry_msgs::PoseStamped
  geometry_msgs::PoseStamped start_hand_pose_stamped_r;
  start_hand_pose_stamped_r.pose = start_hand_pose_r;
  
  // Create the Cartesian trajectories
  trajectory_utils::Cartesian start_traj_r;
  start_traj_r.distal_frame = "RSoftHand";
  start_traj_r.frame = start_hand_pose_stamped_r;
  
  // define the end frame
  geometry_msgs::PoseStamped end_hand_pose_stamped_r;
  tf::poseEigenToMsg (shared_data().sr_hand_pose, end_hand_pose_stamped_r.pose);

  
  trajectory_utils::Cartesian end_r;
  end_r.distal_frame = "RSoftHand";
  end_r.frame = end_hand_pose_stamped_r;


  // define the first segment
  trajectory_utils::segment s_r;
  s_r.type.data = 0;        // min jerk traj
  s_r.T.data = 5.0;         // traj duration 1 second      
  s_r.start = start_traj_r; // start pose
  s_r.end = end_r;          // end pose
  
  // only one segment in this example
  std::vector<trajectory_utils::segment> segments_r;
  segments_r.push_back (s_r);
  
  // prapere the advr_segment_control
  ADVR_ROS::advr_segment_control srv_r;
  srv_r.request.segment_trj.header.frame_id = "world_odom";
  srv_r.request.segment_trj.header.stamp = ros::Time::now();
  srv_r.request.segment_trj.segments = segments_r;
  
  // call the service
  shared_data()._client.call (srv_r);
  }
  }
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Homing::run (double time, double period)
{
  //TBD: Home
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Homing::exit () {}

/*END Homing*/


 /*BEGIN Move_Fail*/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_Fail::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_Fail::entry (const XBot::FSM::Message& msg) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_Fail::run (double time, double period)
{
  std::cout << "Move_Fail run" << std::endl;
  
  //TBD: ungrasp RH
  //TBD: Home RH
  transit("Home");
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_Fail::exit () {}

/*END Move_Fail*/

 /*BEGIN Grasp_Fail*/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_Fail::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_Fail::entry (const XBot::FSM::Message& msg) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_Fail::run (double time, double period)
{
  std::cout << "Grasp_Fail run" << std::endl;
  //TBD: ungrasp RH
  
  transit("Grasp_RH");
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_Fail::exit () {}

/*END Grasp_Fail*/


 /*BEGIN Orient_Fail*/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_Fail::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_Fail::entry (const XBot::FSM::Message& msg) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_Fail::run (double time, double period)
{
  std::cout << "Orient_Fail run" << std::endl;
  
  transit("Grasp_RH_Done");
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_Fail::exit () {}

/*END Orient_Fail*/


 /*BEGIN Push_Fail*/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_Fail::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_Fail::entry (const XBot::FSM::Message& msg) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_Fail::run (double time, double period)
{
  std::cout << "Push_Fail run" << std::endl;
  
  //TBD: Home LH

  transit("Orient_Fail");
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_Fail::exit () {}

/*END Push_Fail*/