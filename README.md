fsm_plugin
===============
Hose

-- How to run:
> roscore
> roslaunch bigman_gazebo bigman_hose_task.launch
> CommunicationHandler config_walkman.yaml 
> NRTDeployer config_walkman.yaml
> XBotGUI config_walkman.yaml

> rostopic pub /hose_grasp_pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 0.6 
    y: 0.5
    z: 0.0
  orientation:
    x: 0.0
    y: -0.7071
    z: 0.0
    w: 0.7071"

>service call /Hose_Task_cmd "success"

> rostopic pub /hose_push_pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 0.5 
    y: -0.4
    z: 0.3
  orientation:
    x: 0.0
    y: -0.7071
    z: 0.0
    w: 0.7071"

> rosservice call /Hose_Task_cmd "success" 
