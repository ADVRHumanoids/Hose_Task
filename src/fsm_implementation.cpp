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
myfsm::Home::entry (const XBot::FSM::Message& msg)
{

}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Home::run (double time, double period)
{
  if (shared_data().do_home_)
  {
    // Home the robot
    localTimer = localTimer >1 ? 1: localTimer;

    // Set the robot's position references taken from its state
    shared_data()._robot->setPositionReference
    (shared_data()._q0+localTimer*(shared_data().state[0]-shared_data()._q0));
    shared_data()._robot->move();

    if (localTimer == 1)
    {
      // Homing finished, so move to the ne state
      shared_data()._robot->getJointPosition(shared_data()._q0);
      localTimer = 0;

      // New state transit
      transit ("Move_LH");
    }
    localTimer += localStep;
  }
  else
  {
    // Move to the next state: Move_LH
    transit ("Home2");
  }
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Home::exit ()
{
  // Info messages
  std::cout << "State: Home::exit" << std::endl;
}
/********************************* END Home **********************************/

/******************************** BEGIN Home *********************************/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Home2::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Home2::entry (const XBot::FSM::Message& msg)
{
  std::cout << "State: Home2 entry" << std::endl;
  // Move LH to LH_Pose (1 mid point in the z-axis fixed dist)

  // Only for first time sense and sync to the model
  //TBD: put flag to check if it is first run to retrieve hands pose
  shared_data()._robot->sense();
  Eigen::Affine3d world_T_bl;
  std::string fb;
  shared_data()._robot->model().getFloatingBaseLink(fb);
  tf.getTransformTf(fb, shared_data ().frame_id_, world_T_bl);
  shared_data()._robot->model().setFloatingBasePose(world_T_bl);
  shared_data()._robot->model().update();

  // Get current hand
  KDL::Frame hand_pose_KDL;
  Eigen::Affine3d hand_pose, r_hand_pose;
  geometry_msgs::Pose start_hand_pose, r_start_hand_pose;

  // Get hands poses
  shared_data()._robot->model().getPose("LSoftHand", hand_pose);
  shared_data().sl_hand_pose = hand_pose;
  shared_data()._robot->model().getPose("RSoftHand", r_hand_pose);
  shared_data().sr_hand_pose = r_hand_pose;

  // Transform from Eigen::Affine3d to geometry_msgs::Pose
  tf::poseEigenToMsg (hand_pose, start_hand_pose);
  tf::poseEigenToMsg (r_hand_pose, r_start_hand_pose);


  // Define the start frame as geometry_msgs::PoseStamped



  // Define the start frame as geometry_msgs::PoseStamped
  geometry_msgs::PoseStamped start_hand_pose_stamped_r;
  start_hand_pose_stamped_r.pose=r_start_hand_pose;

  geometry_msgs::PoseStamped start_hand_pose_stamped_l;
  start_hand_pose_stamped_l.pose=start_hand_pose;

  shared_data()._last_lh_pose =
  boost::shared_ptr<geometry_msgs::PoseStamped>
  (new geometry_msgs::PoseStamped (start_hand_pose_stamped_l));
  shared_data()._last_rh_pose =
  boost::shared_ptr<geometry_msgs::PoseStamped>
  (new geometry_msgs::PoseStamped (start_hand_pose_stamped_r));

  std::cout << "Send \"home_left_hand\" msg..." << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Home2::run (double time, double period)
{

  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str()  << std::endl;

    // LH Move failed
    if (!shared_data().current_command->str().compare("lh_move_fail"))
    transit ("Home2");

    // LH Move Succeeded
    if (!shared_data().current_command->str().compare("home_left_hand"))
    transit ("Home_LH");

  }

}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Home2::exit ()
{
  // Info messages
  std::cout << "State: Home::exit" << std::endl;
}
/********************************* END Home **********************************/


/******************************* BEGIN Move_LH *******************************/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_LH::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_LH::entry (const XBot::FSM::Message& msg)
{
  std::cout << "Pub /hose_pose for left hand..." << std::endl;

  ADVR_ROS::im_pose_msg::ConstPtr tmp;
  tmp = ros::topic::waitForMessage<ADVR_ROS::im_pose_msg>(shared_data ().pose_cmd_);
  shared_data()._hose_grasp_pose =  boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(tmp->pose_stamped));


  geometry_msgs::PoseStamped start_hand_pose_stamped=*shared_data()._last_lh_pose;

  // Create the Cartesian trajectories
  trajectory_utils::Cartesian start_traj1;
  start_traj1.distal_frame = "LSoftHand";
  start_traj1.frame = start_hand_pose_stamped;

  // define the end frame
  geometry_msgs::PoseStamped end_hand_pose_stamped1;

  end_hand_pose_stamped1.pose.position =
  shared_data()._hose_grasp_pose->pose.position;
  end_hand_pose_stamped1.pose.position.z += 0.2;

  end_hand_pose_stamped1.pose.orientation =
  shared_data()._hose_grasp_pose->pose.orientation;

  /*end_hand_pose_stamped.pose.orientation.x =  0.094453573229;
  end_hand_pose_stamped.pose.orientation.y = -0.324954947790;
  end_hand_pose_stamped.pose.orientation.z = -0.601136949013;
  end_hand_pose_stamped.pose.orientation.w =  0.723959372439;
  */
  trajectory_utils::Cartesian end1;
  end1.distal_frame = "LSoftHand";
  end1.frame = end_hand_pose_stamped1;

  // define the first segment
  trajectory_utils::segment s11;
  s11.type.data = 0;        // min jerk traj
  s11.T.data = 10.0;        // traj duration 1 second
  s11.start = start_traj1;   // start pose
  s11.end = end1;            // end pose

  start_traj1.frame = end_hand_pose_stamped1;

  end_hand_pose_stamped1.pose.position.z =
  shared_data()._hose_grasp_pose->pose.position.z;
  end1.frame = end_hand_pose_stamped1;

  // define the first segment
  trajectory_utils::segment s2;
  s2.type.data = 0;        // min jerk traj
  s2.T.data = 10.0;        // traj duration 1 second
  s2.start = start_traj1;   // start pose
  s2.end = end1;            // end pose

  // only one segment in this example
  std::vector<trajectory_utils::segment> segments1;
  segments1.push_back (s11);
  segments1.push_back (s2);

  // prapere the advr_segment_control
  ADVR_ROS::advr_segment_control srv1;
  srv1.request.segment_trj.header.frame_id = shared_data ().frame_id_;
  srv1.request.segment_trj.header.stamp = ros::Time::now();
  srv1.request.segment_trj.segments = segments1;

  // call the service
  shared_data()._client.call(srv1);

  // save last hand pose
  shared_data()._last_lh_pose =
  boost::shared_ptr<geometry_msgs::PoseStamped>
  (new geometry_msgs::PoseStamped (end_hand_pose_stamped1));
  // Info msg
  std::cout << "Send \"lh_move_fail\" or \"success\" msg..." << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_LH::run (double time, double period)
{
  // Blocking Reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str()  << std::endl;

    // LH Move failed
    if (!shared_data().current_command->str().compare("lh_move_fail"))
    transit ("Move_LH");

    // LH Move Succeeded
    if (!shared_data().current_command->str().compare("success"))
    transit ("Grasp_LH");

    if (!shared_data().current_command->str().compare ("home_left_hand"))
    transit("Home_LH");
  }
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_LH::exit ()
{
  std::cout << "State: Move_LH::exit" << std::endl;
}
/******************************** END Move_LH ********************************/

/*BEGIN Grasp_LH*/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_LH::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_LH::entry (const XBot::FSM::Message& msg)
{
  std::cout << "State: Grasp_LH::entry" << std::endl;

  ADVR_ROS::advr_grasp_control_srv srv;
  srv.request.right_grasp = 1.0; // right hand: opened
  srv.request.left_grasp = 1.0;  // left hand: closed
  shared_data ()._grasp_client.call (srv); // call the service

  // Info msg
  std::cout << "Send \"lh_grasp_fail\" or \"success\" msg..." << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_LH::run (double time, double period)
{
  // Blocking Reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    //std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // LH Grasped failed
    if (!shared_data().current_command->str().compare("lh_grasp_fail"))
    {
      // ungrasp first
      ADVR_ROS::advr_grasp_control_srv srv;
      srv.request.right_grasp = 0.0; // right hand: opened
      srv.request.left_grasp = 0.0;  // left hand: opened
      shared_data ()._grasp_client.call (srv); // call the service
      transit ("Move_LH");
    }
    // LH Grasped Succeeded
    if (!shared_data().current_command->str().compare("success"))
    transit ("Grasp_LH_Done");

    if (!shared_data().current_command->str().compare ("home_left_hand"))
    transit("Home_LH");
  }
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_LH::exit ()
{
  std::cout << "State: Grasp_LH::exit" << std::endl;
}
/******************************** END Grasp_LH ********************************/


/*BEGIN Grasp_LH_Done*/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_LH_Done::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_LH_Done::entry (const XBot::FSM::Message& msg)
{
  std::cout << "State: Grasp_LH_Done::entry" << std::endl;

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
  srv.request.segment_trj.header.frame_id = shared_data ().frame_id_;
  srv.request.segment_trj.header.stamp = ros::Time::now();
  srv.request.segment_trj.segments = segments;

  // call the service
  shared_data()._client.call(srv);

  // save last hand pose
  shared_data()._last_lh_pose =
  boost::shared_ptr<geometry_msgs::PoseStamped>
  (new geometry_msgs::PoseStamped (end_hand_pose_stamped));

  // Info msg
  std::cout << "Send \"success\" msg..." << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_LH_Done::run (double time, double period)
{
  // Blocking Reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data ().current_command->str() << std::endl;

    //TBD: failure cases
    // RH Move failed
    //bool grasp_rh_fail = false;
    //bool move_rh_fail = false;

    //if (grasp_rh_fail)
    //  transit("Grasp_Fail");

    //if (move_rh_fail)
    //  transit("Move_Fail");

    // RH Move Succeeded
    if (!shared_data().current_command->str().compare ("success"))
    transit("Move_RH");
    if (!shared_data().current_command->str().compare ("home_left_hand"))
    transit("Home_LH");
  }
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Grasp_LH_Done::exit ()
{
  std::cout << "State: Grasp_LH_Done::exit" << std::endl;
}
/***************************** END Grasp_LH_Done *****************************/


/****************************** BEGIN Orient_LH ******************************/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_LH::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_LH::entry (const XBot::FSM::Message& msg)
{
  std::cout << "State: Orient_LH::entry" << std::endl;
  std::cout << "Pub /hose_pose for left hand orientation..." << std::endl;

  //Wait for RH_Pose (orientation)
  ADVR_ROS::im_pose_msg::ConstPtr tmp;
  tmp = ros::topic::waitForMessage<ADVR_ROS::im_pose_msg>(shared_data ().pose_cmd_);
  shared_data()._hose_grasp_pose =  boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(tmp->pose_stamped));


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

  end_hand_pose_stamped.pose.orientation =
  shared_data()._hose_grasp_pose->pose.orientation;
  end_hand_pose_stamped.pose.position =
  shared_data()._hose_grasp_pose->pose.position;

  trajectory_utils::Cartesian end;
  end.distal_frame = "LSoftHand";
  end.frame = end_hand_pose_stamped;

  // define the first segment
  trajectory_utils::segment s1;
  s1.type.data = 0;        // min jerk traj
  s1.T.data = 5.0;         // traj duration
  s1.start = start_traj;   // start pose
  s1.end = end;            // end pose

  // only one segment in this example
  std::vector<trajectory_utils::segment> segments;
  segments.push_back (s1);

  // prapere the advr_segment_control
  ADVR_ROS::advr_segment_control srv;
  srv.request.segment_trj.header.frame_id = shared_data ().frame_id_;
  srv.request.segment_trj.header.stamp = ros::Time::now();
  srv.request.segment_trj.segments = segments;

  // call the service
  shared_data()._client.call(srv);

  // save last hand pose
  shared_data()._last_lh_pose =
  boost::shared_ptr<geometry_msgs::PoseStamped>
  (new geometry_msgs::PoseStamped (end_hand_pose_stamped));

  // Info msg
  std::cout << "Send \"success\" msg..." << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_LH::run (double time, double period)
{
  // Blocking Reading: wait for a command
  if(!shared_data().current_command->str().empty() )
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // RH Move Succeeded
    if (!shared_data().current_command->str().compare("success"))
    transit("Orient_LH_Done");

    if (!shared_data().current_command->str().compare("lh_orient_fail"))
    transit("Orient_LH");

    if (!shared_data().current_command->str().compare ("home_left_hand"))
    transit("Home_LH");
  }
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_LH::exit ()
{
  std::cout << "State: Orient_LH::exit" << std::endl;
}
/******************************* END Orient_LH *******************************/


/*************************** BEGIN Orient_LH_Done ****************************/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_LH_Done::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_LH_Done::entry (const XBot::FSM::Message& msg)
{
  std::cout << "State: Orient_LH_Done::entry" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_LH_Done::run (double time, double period)
{

  //TBD: Check if the RH orientation has failed
  //bool orient_fail = false;
  //if (orient_fail)
  //  transit("Orient_Fail");

  transit("Push_RH");
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Orient_LH_Done::exit ()
{
  std::cout << "State: Orient_LH_Done::exit" << std::endl;
}
/**************************** END Orient_LH_Done *****************************/


/******************************* BEGIN Move_RH *******************************/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_RH::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_RH::entry (const XBot::FSM::Message& msg)
{
  std::cout << "State: Move_RH::entry" << std::endl;
  std::cout << "Pub /hose_pose for right hand orientation..." << std::endl;

  //Wait RH_Pose
  ADVR_ROS::im_pose_msg::ConstPtr tmp;
  tmp = ros::topic::waitForMessage<ADVR_ROS::im_pose_msg>(shared_data ().pose_cmd_);
  shared_data()._hose_grasp_pose =  boost::shared_ptr<geometry_msgs::PoseStamped>(new geometry_msgs::PoseStamped(tmp->pose_stamped));

  // Define the start frame as geometry_msgs::PoseStamped
  geometry_msgs::PoseStamped start_hand_pose_stamped;
  start_hand_pose_stamped = *shared_data()._last_rh_pose;

  // Create the Cartesian trajectories
  trajectory_utils::Cartesian start_traj;
  start_traj.distal_frame = "RSoftHand";
  start_traj.frame = start_hand_pose_stamped;

  // define the end frame
  geometry_msgs::PoseStamped end_hand_pose_stamped;
  end_hand_pose_stamped.pose.position =
  shared_data()._hose_grasp_pose->pose.position;

  std::cout << "end_hand_pose_stamped.pose.position: "
  << end_hand_pose_stamped.pose.position.x << ","
  << end_hand_pose_stamped.pose.position.y << ","
  << end_hand_pose_stamped.pose.position.z << std::endl;
  end_hand_pose_stamped.pose.position.x/=1.5;
  end_hand_pose_stamped.pose.position.y/=1.5;
  end_hand_pose_stamped.pose.position.z+=0.15;
  end_hand_pose_stamped.pose.orientation.x = 0.0;
  end_hand_pose_stamped.pose.orientation.y = -0.7071;
  end_hand_pose_stamped.pose.orientation.z = 0.0;
  end_hand_pose_stamped.pose.orientation.w = 0.7071;

  trajectory_utils::Cartesian end;
  end.distal_frame = "RSoftHand";
  end.frame = end_hand_pose_stamped;

  // define the first segment
  trajectory_utils::segment s1;
  s1.type.data = 0;        // min jerk traj
  s1.T.data = 5.0;         // traj duration
  s1.start = start_traj;   // start pose
  s1.end = end;            // end pose

  start_traj.frame = end_hand_pose_stamped;
  end_hand_pose_stamped.pose.position =
  shared_data()._hose_grasp_pose->pose.position;
  end.frame = end_hand_pose_stamped;

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
  srv.request.segment_trj.header.frame_id = shared_data ().frame_id_;
  srv.request.segment_trj.header.stamp = ros::Time::now();
  srv.request.segment_trj.segments = segments;

  // call the service
  shared_data()._client.call(srv);

  // save last hand pose
  shared_data()._last_rh_pose =
  boost::shared_ptr<geometry_msgs::PoseStamped>
  (new geometry_msgs::PoseStamped (end_hand_pose_stamped));

  std::cout << "saved end_hand_pose_stamped.pose.position: "
  << end_hand_pose_stamped.pose.position.x << ","
  << end_hand_pose_stamped.pose.position.y << ","
  << end_hand_pose_stamped.pose.position.z << std::endl;

  // Info msg
  std::cout << "Send \"success\" msg..." << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_RH::run (double time, double period)
{
  // Blocking Reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // LH Move Succeeded
    if (!shared_data().current_command->str().compare ("success"))
    transit("Orient_LH");
    if (!shared_data().current_command->str().compare("rh_move_fail"))
    transit("Move_RH");
    if (!shared_data().current_command->str().compare ("home_left_hand"))
    transit("Home_LH");
    if (!shared_data().current_command->str().compare ("home_right_hand"))
    transit("Home_RH");
  }
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Move_RH::exit ()
{
  std::cout << "State: Move_RH::exit" << std::endl;
}
/******************************* END Move_RH ********************************/


/******************************* BEGIN Push_RH *******************************/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_RH::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_RH::entry (const XBot::FSM::Message& msg)
{
  std::cout << "State: Push_RH::entry" << std::endl;

  // Info msg
  std::cout << "Send \"success\" msg..." << std::endl;

  // Define the start frame as geometry_msgs::PoseStamped
  geometry_msgs::PoseStamped start_hand_pose_stamped;
  start_hand_pose_stamped = *shared_data()._last_rh_pose;

  std::cout << "loaded start_hand_pose_stamped.pose.position: "
  << start_hand_pose_stamped.pose.position.x << ","
  << start_hand_pose_stamped.pose.position.y << ","
  << start_hand_pose_stamped.pose.position.z << std::endl;

  // Create the Cartesian trajectories
  trajectory_utils::Cartesian start_traj;
  start_traj.distal_frame = "RSoftHand";
  start_traj.frame = start_hand_pose_stamped;

  // define the end frame
  geometry_msgs::PoseStamped end_hand_pose_stamped;
  end_hand_pose_stamped.pose.position =
  start_hand_pose_stamped.pose.position;

  end_hand_pose_stamped.pose.orientation.x = 0.0;
  end_hand_pose_stamped.pose.orientation.y = -0.7071;
  end_hand_pose_stamped.pose.orientation.z = 0.0;
  end_hand_pose_stamped.pose.orientation.w = 0.7071;

  trajectory_utils::Cartesian end_traj;
  end_traj.distal_frame = "RSoftHand";
  end_traj.frame = end_hand_pose_stamped;

  // define the first segment
  std::vector<trajectory_utils::segment> segments;
  trajectory_utils::segment s1;
  s1.type.data = 0;        // min jerk traj
  s1.T.data = 3.0;         // traj duration

  //oly for simulation
  //ADVR_ROS::advr_grasp_control_srv grasp_srv;
  //grasp_srv.request.right_grasp = 0.0;
  //grasp_srv.request.left_grasp = 1.0;
  // call the service
  shared_data ()._grasp_client.call(grasp_srv);


  for (int i=0; i<6; i++)
  {
    //start_traj.distal_frame = "RSoftHand";
    start_traj.frame = end_hand_pose_stamped;

    std::cout << "S end_frame.pose.position.z: "
    << end_hand_pose_stamped.pose.position.z
    << std::endl;
    end_hand_pose_stamped.pose.position.z -= 0.10*pow(-1,i);
    std::cout << "F end_frame.pose.position.z: "
    << end_hand_pose_stamped.pose.position.z
    << std::endl;
    end_traj.frame = end_hand_pose_stamped;

    s1.start = start_traj;   // start pose
    s1.end = end_traj;       // end pose
    segments.push_back (s1);
  }

  // prapere the advr_segment_control
  ADVR_ROS::advr_segment_control srv;
  srv.request.segment_trj.header.frame_id = shared_data ().frame_id_;
  srv.request.segment_trj.header.stamp = ros::Time::now();
  srv.request.segment_trj.segments = segments;

  // call the service
  shared_data()._client.call(srv);

  // save last hand pose
  shared_data()._last_rh_pose =
  boost::shared_ptr<geometry_msgs::PoseStamped>
  (new geometry_msgs::PoseStamped (end_hand_pose_stamped));
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_RH::run (double time, double period)
{
  //TBD: check if the orientation or mnove has failed
  bool orient_fail = false;
  bool push_fail = false;

  if (push_fail)
  transit("Push_Fail");
  else if (orient_fail)
  transit("Orient_Fail");

  // Blocking Reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // LH Move Succeeded
    if (!shared_data().current_command->str().compare ("rh_push_failed"))
    transit("Move_RH");
    if (!shared_data().current_command->str().compare ("lh_orient_fail"))
    transit("Orient_LH");
    if (!shared_data().current_command->str().compare ("success"))
    transit("Push_RH_Done");
    if (!shared_data().current_command->str().compare ("home_left_hand"))
    transit("Home_LH");
    if (!shared_data().current_command->str().compare ("home_right_hand"))
    transit("Home_RH");
  }
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_RH::exit ()
{
  std::cout << "State: Push_RH::exit" << std::endl;
}

/*END Push_RH*/


/*BEGIN Push_RH_Done*/
///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_RH_Done::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_RH_Done::entry (const XBot::FSM::Message& msg)
{
  std::cout << "State: Push_RH_Done::entry" << std::endl;
  ADVR_ROS::advr_grasp_control_srv srv;
  srv.request.right_grasp = 0.0;
  srv.request.left_grasp = 0.0;
  // call the service
  shared_data ()._grasp_client.call(srv);
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_RH_Done::run (double time, double period)
{
  // Blocking Reading: wait for a command
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // LH Move Succeeded
    if (!shared_data().current_command->str().compare ("success"))
    {
      transit("Home_LH");
      shared_data().task_complete_=true;
    }
    if (!shared_data().current_command->str().compare ("home_left_hand"))
    transit("Home_LH");
    if (!shared_data().current_command->str().compare ("home_right_hand"))
    transit("Home_RH");
  }

  //TBD: Ungrasp both hands


  //transit("Homing");
}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Push_RH_Done::exit ()
{
  ADVR_ROS::advr_grasp_control_srv srv;
  srv.request.right_grasp = 0.0;
  srv.request.left_grasp = 0.0;
  // call the service
  shared_data ()._grasp_client.call(srv);
}

/*END Push_RH_Done*/


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

  // Define the start frame as geometry_msgs::PoseStamped
  geometry_msgs::PoseStamped start_hand_pose_stamped=*shared_data()._last_lh_pose;


  // Create the Cartesian trajectories
  trajectory_utils::Cartesian start_traj;
  start_traj.distal_frame = "LSoftHand";
  start_traj.frame = start_hand_pose_stamped;


  geometry_msgs::PoseStamped end_frame_l;

  end_frame_l.pose.position.x = 0.248;
  end_frame_l.pose.position.y = 0.521;
  end_frame_l.pose.position.z = 1.169;

  end_frame_l.pose.orientation.x = 0.091;
  end_frame_l.pose.orientation.y = -0.456;
  end_frame_l.pose.orientation.z = -0.19;
  end_frame_l.pose.orientation.w = 0.864;


  trajectory_utils::Cartesian end;
  end.distal_frame = "LSoftHand";
  end.frame = end_frame_l;


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
  srv.request.segment_trj.header.frame_id = shared_data ().frame_id_;
  srv.request.segment_trj.header.stamp = ros::Time::now();
  srv.request.segment_trj.segments = segments;

  // call the service
  shared_data()._client.call(srv);

  std::cout << "Send \"success\" msg..." << std::endl;


  // Define the start frame as geometry_msgs::PoseStamped
  geometry_msgs::PoseStamped start_hand_pose_stamped_r=*shared_data()._last_rh_pose;

  // Create the Cartesian trajectories
  trajectory_utils::Cartesian start_traj_r;
  start_traj_r.distal_frame = "RSoftHand";
  start_traj_r.frame = start_hand_pose_stamped_r;

  // define the end frame

  geometry_msgs::PoseStamped end_frame_r;
  //     end_frame = *shared_data()._initial_pose_right_hand;
  end_frame_r.pose.position.x = 0.248;
  end_frame_r.pose.position.y = -0.521;
  end_frame_r.pose.position.z = 0.969;

  end_frame_r.pose.orientation.x = -0.091;
  end_frame_r.pose.orientation.y = -0.456;
  end_frame_r.pose.orientation.z = 0.19;
  end_frame_r.pose.orientation.w = 0.864;


  trajectory_utils::Cartesian end_r;
  end_r.distal_frame = "RSoftHand";
  end_r.frame = end_frame_r;


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
  srv_r.request.segment_trj.header.frame_id = shared_data ().frame_id_;
  srv_r.request.segment_trj.header.stamp = ros::Time::now();
  srv_r.request.segment_trj.segments = segments_r;

  // call the service
  shared_data()._client.call (srv_r);


}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Homing::run (double time, double period)
{
  //TBD: Home
  ADVR_ROS::advr_grasp_control_srv srv;
  srv.request.right_grasp = 0.0;
  srv.request.left_grasp = 0.0;
  // call the service
  shared_data ()._grasp_client.call(srv);
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

  transit("Grasp_LH");
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

  transit("Grasp_LH_Done");
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

void myfsm::Push_Fail::exit (){};

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Home_LH::exit () {}

void
myfsm::Home_LH::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Home_LH::entry (const XBot::FSM::Message& msg)
{
  ADVR_ROS::advr_grasp_control_srv grasp_srv;
  grasp_srv.request.left_grasp = 0.0; // left hand: opened
  shared_data ()._grasp_client.call (grasp_srv); // call the service

  geometry_msgs::PoseStamped start_hand_pose_stamped;
  start_hand_pose_stamped = *shared_data()._last_lh_pose;

  // Create the Cartesian trajectories
  trajectory_utils::Cartesian start_traj;
  start_traj.distal_frame = "LSoftHand";
  start_traj.frame = start_hand_pose_stamped;

  // define the end frame



  geometry_msgs::PoseStamped end_frame_l;

  end_frame_l.pose.position.x = 0.248;
  end_frame_l.pose.position.y = 0.521;
  end_frame_l.pose.position.z = 1.169;

  end_frame_l.pose.orientation.x = 0.091;
  end_frame_l.pose.orientation.y = -0.456;
  end_frame_l.pose.orientation.z = -0.19;
  end_frame_l.pose.orientation.w = 0.864;



  //end_hand_pose_stamped.pose.orientation =
  //  shared_data()._hose_grasp_pose->pose.orientation;

  trajectory_utils::Cartesian end;
  end.distal_frame = "LSoftHand";
  end.frame = end_frame_l;

  // define the first segment
  trajectory_utils::segment s1;
  s1.type.data = 0;        // min jerk traj
  s1.T.data = 5.0;         // traj duration
  s1.start = start_traj;   // start pose
  s1.end = end;            // end pose

  // only one segment in this example
  std::vector<trajectory_utils::segment> segments;
  segments.push_back (s1);

  // prapere the advr_segment_control
  ADVR_ROS::advr_segment_control srv;
  srv.request.segment_trj.header.frame_id = shared_data ().frame_id_;
  srv.request.segment_trj.header.stamp = ros::Time::now();
  srv.request.segment_trj.segments = segments;

  // call the service
  shared_data()._client.call(srv);

  // save last hand pose
  shared_data()._last_lh_pose =
  boost::shared_ptr<geometry_msgs::PoseStamped>
  (new geometry_msgs::PoseStamped (end_frame_l));

}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Home_LH::run (double time, double period)
{
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // Home RH Succeeded
    if (!shared_data().current_command->str().compare ("success") )
      transit("Move_LH");
    if (!shared_data().current_command->str().compare ("home_right_hand"))
      transit("Home_RH");
  }

}


/*END Home_LH*/

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Home_RH::exit () {}

void
myfsm::Home_RH::react (const XBot::FSM::Event& e) {}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Home_RH::entry (const XBot::FSM::Message& msg)
{
  ADVR_ROS::advr_grasp_control_srv grasp_srv;
  grasp_srv.request.right_grasp = 0.0; // right hand: opened
  shared_data ()._grasp_client.call (grasp_srv); // call the service

  geometry_msgs::PoseStamped start_hand_pose_stamped;
  start_hand_pose_stamped = *shared_data()._last_rh_pose;

  // Create the Cartesian trajectories
  trajectory_utils::Cartesian start_traj;
  start_traj.distal_frame = "RSoftHand";
  start_traj.frame = start_hand_pose_stamped;

  // define the end frame


  geometry_msgs::PoseStamped end_frame_r;
  //     end_frame = *shared_data()._initial_pose_right_hand;
  end_frame_r.pose.position.x = 0.248;
  end_frame_r.pose.position.y = -0.521;
  end_frame_r.pose.position.z = 1.169;

  end_frame_r.pose.orientation.x = -0.091;
  end_frame_r.pose.orientation.y = -0.456;
  end_frame_r.pose.orientation.z = 0.19;
  end_frame_r.pose.orientation.w = 0.864;




  //end_hand_pose_stamped.pose.orientation =
  //  shared_data()._hose_grasp_pose->pose.orientation;

  trajectory_utils::Cartesian end;
  end.distal_frame = "RSoftHand";
  end.frame = end_frame_r;

  // define the first segment
  trajectory_utils::segment s1;
  s1.type.data = 0;        // min jerk traj
  s1.T.data = 5.0;         // traj duration
  s1.start = start_traj;   // start pose
  s1.end = end;            // end pose

  // only one segment in this example
  std::vector<trajectory_utils::segment> segments;
  segments.push_back (s1);

  // prapere the advr_segment_control
  ADVR_ROS::advr_segment_control srv;
  srv.request.segment_trj.header.frame_id = shared_data ().frame_id_;
  srv.request.segment_trj.header.stamp = ros::Time::now();
  srv.request.segment_trj.segments = segments;

  // call the service
  shared_data()._client.call(srv);

  // save last hand pose
  shared_data()._last_rh_pose =
  boost::shared_ptr<geometry_msgs::PoseStamped>
  (new geometry_msgs::PoseStamped (end_frame_r));

}

///////////////////////////////////////////////////////////////////////////////
void
myfsm::Home_RH::run (double time, double period)
{
  if(!shared_data().current_command->str().empty())
  {
    std::cout << "Command: " << shared_data().current_command->str() << std::endl;

    // RH home succeeded Succeeded
    if (!shared_data().current_command->str().compare ("success"))
    {
      if(shared_data().do_home_)
      {
        if(!shared_data().task_complete_)
         transit("Move_RH");
      }
      else
      {
        transit("Move_LH");
        shared_data().do_home_=true;
      }
    }
  }

}


/*END Home_LH*/
