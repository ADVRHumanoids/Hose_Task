/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
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

#include <Hose_Task_rt_plugin.h>

/* Specify that the class XBotPlugin::Hose_Task is a XBot RT plugin with name "Hose_Task" */
REGISTER_XBOT_PLUGIN(Hose_Task, XBotPlugin::Hose_Task)

namespace XBotPlugin {

bool Hose_Task::init_control_plugin(std::string path_to_config_file,
                                                    XBot::SharedMemory::Ptr shared_memory,
                                                    XBot::RobotInterface::Ptr robot)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


    /* Save robot to a private member. */
    _robot = robot;

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */
    
    _logger = XBot::MatLogger::getLogger("/tmp/Hose_Task_log");


    /*Saves robot as shared variable between states*/
    fsm.shared_data()._robot= robot;
    
    /*Registers states*/
    fsm.register_state(std::make_shared<myfsm::Home>());
    fsm.register_state(std::make_shared<myfsm::Move_RH>());
    fsm.register_state(std::make_shared<myfsm::Grasp_RH>());
    fsm.register_state(std::make_shared<myfsm::Grasp_RH_Done>());
    fsm.register_state(std::make_shared<myfsm::Orient_RH>());
    fsm.register_state(std::make_shared<myfsm::Orient_RH_Done>());
    fsm.register_state(std::make_shared<myfsm::Move_LH>());
    fsm.register_state(std::make_shared<myfsm::Push_LH>());
    fsm.register_state(std::make_shared<myfsm::Push_LH_Done>());
    fsm.register_state(std::make_shared<myfsm::Homing>());
    fsm.register_state(std::make_shared<myfsm::Move_Fail>());
    fsm.register_state(std::make_shared<myfsm::Grasp_Fail>());
    fsm.register_state(std::make_shared<myfsm::Orient_Fail>());
    fsm.register_state(std::make_shared<myfsm::Push_Fail>());
    
    // Initialize the FSM with the initial state
    fsm.init("Home");

    return true;


}

void Hose_Task::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /Hose_Task_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Save the plugin starting time to a class member */
    _robot->getMotorPosition(_q0);

    /* Save the robot starting config to a class member */
    _start_time = time;
}

void Hose_Task::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /Hose_Task_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
}


void Hose_Task::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */


     fsm.run(time, 0.01);

}

bool Hose_Task::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}



}
