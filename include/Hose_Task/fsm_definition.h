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
#include <XBotInterface/StateMachine.h>

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <ADVR_ROS/advr_segment_control.h>

#include <trajectory_utils/segment.h>
#include <trajectory_utils/Cartesian.h>

#include <eigen_conversions/eigen_msg.h>
#include <XBotCore-interfaces/XDomainCommunication.h>

#include "int_markers_to_pose_array.h"


namespace myfsm
{
  struct SharedData
  {
    Eigen::VectorXd  state[4];

    XBot::RobotInterface::Ptr _robot;

    Eigen::VectorXd _q0;
    
    ros::ServiceClient _client;
    geometry_msgs::PoseStamped::ConstPtr _hose_grasp_pose;
    
    // Grasping publisher
    ros::Publisher _grasp_mag_pub;

    // Input commands
    XBot::SubscriberRT<XBot::Command> command;
    XBot::Command current_command;
    
    // Starting left and right hand poses
    Eigen::Affine3d sl_hand_pose, sr_hand_pose;
    // Interactive marker to stambed pose
    //IntMarkersToPoseArray imtpa;
  };
  
  class MacroState : public  XBot::FSM::State< MacroState , SharedData >
  {
    public:
      virtual void entry(const XBot::FSM::Message& msg) {};
      virtual void react(const XBot::FSM::Event& e){};
  };  

  /** \brief This class state is about homing the robot. */
  class Home : public MacroState
  {
    virtual std::string get_name() const { return "Home"; }

    virtual void run(double time, double period);

    virtual void entry(const XBot::FSM::Message& msg);

    virtual void react(const XBot::FSM::Event& e);

    virtual void exit ();

    private:
      double localTimer = 0;
      double localStep = 0.001;  
  };

  class Move_RH : public MacroState
  {
    virtual std::string get_name() const { return "Move_RH"; }

    virtual void run(double time, double period);

    virtual void entry(const XBot::FSM::Message& msg);

    virtual void react(const XBot::FSM::Event& e);

    virtual void exit ();
  };

  class Grasp_RH : public MacroState
  {
    virtual std::string get_name() const { return "Grasp_RH"; }

    virtual void run(double time, double period);

    virtual void entry(const XBot::FSM::Message& msg);

    virtual void react(const XBot::FSM::Event& e);

    virtual void exit ();
  };

  class Grasp_RH_Done : public MacroState
  {
    virtual std::string get_name() const { return "Grasp_RH_Done"; }

    virtual void run(double time, double period);

    virtual void entry(const XBot::FSM::Message& msg);

    virtual void react(const XBot::FSM::Event& e);

    virtual void exit ();
  };

  class Orient_RH : public MacroState
  {
    virtual std::string get_name() const { return "Orient_RH"; }

    virtual void run(double time, double period);

    virtual void entry(const XBot::FSM::Message& msg);

    virtual void react(const XBot::FSM::Event& e);

    virtual void exit ();
  };

  class Orient_RH_Done : public MacroState
  {
    virtual std::string get_name() const { return "Orient_RH_Done"; }

    virtual void run(double time, double period);

    virtual void entry(const XBot::FSM::Message& msg);

    virtual void react(const XBot::FSM::Event& e);

    virtual void exit ();
  };

  class Move_LH : public MacroState
  {
    virtual std::string get_name() const { return "Move_LH"; }

    virtual void run(double time, double period);

    virtual void entry(const XBot::FSM::Message& msg);

    virtual void react(const XBot::FSM::Event& e);

    virtual void exit ();
  };

  class Push_LH : public MacroState
  {
    virtual std::string get_name() const { return "Push_LH"; }

    virtual void run(double time, double period);

    virtual void entry(const XBot::FSM::Message& msg);

    virtual void react(const XBot::FSM::Event& e);

    virtual void exit ();
  };

  class Push_LH_Done : public MacroState
  {
    virtual std::string get_name() const { return "Push_LH_Done"; }

    virtual void run(double time, double period);

    virtual void entry(const XBot::FSM::Message& msg);

    virtual void react(const XBot::FSM::Event& e);

    virtual void exit ();
  };

  class Homing : public MacroState
    {
    virtual std::string get_name() const { return "Homing"; }

    virtual void run(double time, double period);

    virtual void entry(const XBot::FSM::Message& msg);

    virtual void react(const XBot::FSM::Event& e);

    virtual void exit ();
  };
  
  class Move_Fail : public MacroState
  {
    virtual std::string get_name() const { return "Move_Fail"; }

    virtual void run(double time, double period);

    virtual void entry(const XBot::FSM::Message& msg);

    virtual void react(const XBot::FSM::Event& e);

    virtual void exit ();
  };
    
  class Grasp_Fail : public MacroState
  {
    virtual std::string get_name() const { return "Grasp_Fail"; }

    virtual void run(double time, double period);

    virtual void entry(const XBot::FSM::Message& msg);

    virtual void react(const XBot::FSM::Event& e);

    virtual void exit ();      
  };

  class Orient_Fail : public MacroState
  {
    virtual std::string get_name() const { return "Orient_Fail"; }

    virtual void run(double time, double period);

    virtual void entry(const XBot::FSM::Message& msg);

    virtual void react(const XBot::FSM::Event& e);

    virtual void exit ();      
  };


  class Push_Fail : public MacroState
  {
    virtual std::string get_name() const { return "Push_Fail"; }

    virtual void run(double time, double period);

    virtual void entry(const XBot::FSM::Message& msg);

    virtual void react(const XBot::FSM::Event& e);

    virtual void exit ();
  };
}