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

#ifndef Hose_Task_H_
#define Hose_Task_PLUGIN_H_

#include <XCM/XBotControlPlugin.h>
#include "fsm_definition.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <ADVR_ROS/advr_segment_control.h>

#include <trajectory_utils/segment.h>
#include <trajectory_utils/Cartesian.h>

#include "int_markers_to_pose_array.h"

namespace XBotPlugin
{
  /** \brief Hose_Task XBot RT Plugin **/
  class Hose_Task : public XBot::XBotControlPlugin
  {
    public:
      virtual bool init_control_plugin(XBot::Handle::Ptr handle);

      virtual bool close();

      virtual void on_start(double time);

      virtual void on_stop(double time);

    protected:
      virtual void control_loop(double time, double period);

    private:
      XBot::RobotInterface::Ptr _robot;

      double _start_time;

      //Eigen::VectorXd _q0;

      XBot::MatLogger::Ptr _logger;

      XBot::FSM::StateMachine< myfsm::MacroState , myfsm::SharedData > fsm;

      std::shared_ptr<ros::NodeHandle> _nh;
  };
}

#endif // Hose_Task_PLUGIN_H_
