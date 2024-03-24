# TMS-I/F-For-OPERA

The tms_if_for_opera module serves as an interface that mediates the connection between the ROS2-TMS for Construction developed by us and the OPERA developed by PWRI. 
The following figure is used overall architecture.


![](docs/tms_if_for_opera.png)


As shown in the figure above, the tms_if_for_opera is an interface module used to connect ROS2-TMS for Construction and OPERA. It operates the OPERA-compatible construction machinery based on the operation instructions sent from the task management sysytem of ROS2-TMS for Construction.

Currently, the tms_if_for_opera module is implemented only with the functionality to interface with the OPERA-compatible construction machine ZX200, and the operational overview is depicted in the figure above.
The operation will be explained below.


1. First, the task management system of ROS2-TMS for Construction executes sub-tasks according to the task sequence. Then, based on its implementation, the sub-task sends data required for motion planning, such as target position and orientation for the desired joint (e.g., end effector) of the OPERA-compatible construction machinery, to tms_if_for_opera using MoveIt!.
2. When tms_if_for_opera receives these values from the Subtask Node executed by the task management system of ROS2 TMS for Construction, it forwards those values to move_group, which is part of OPERA.
3. Upon receiving the values, the move_group module performs motion planning using MoveIt!, and the planned motion plan is then sent to modules referred to as JointTrajectoryController or HardwareInterface.
4. Then, the JointTrajectoryController or HardwareInterface calculates values such as valve openings to be sent to the actual construction machinery, based on the received motion plan, to faithfully execute these commands on the OPERA-compatible construction machinery in real life. These calculated values are sent to the excavator_com3_ros module.
5. Up to this point, the communication flow from Subtask Node → move_group_interface → move_group → JointTrajectoryController → HardwareInterface → excavator_com3_ros has all been conducted using ROS2 Actions and based on the ROS2 format. However, when it comes to actually operating the construction machinery, it's necessary to convert the ROS2 communication into CAN signals. This function is handled by excavator_com3_ros.
6. The CAN signals calculated by the excavator_com3_ros module are then sent to the actual OPERA-compatible construction machinery, initiating operation. Furthermore, state signals included in the current status of each joint are sent from the OPERA-compatible construction machinery back to excavator_com3_ros as CAN signals.
6. Then, excavator_com3_ros converts the received CAN signals into ROS2 format and returns the values in reverse order from earlier: excavator_com3_ros → HardwareInterface → JointTrajectoryController → move_group → tms_if_for_opera → Subtask Node.
7. And then, the Subtask Node transmits these signals, which indicate the current state of OPERA, to the task management system of ROS2-TMS for Construction. By doing so, the task management systenm of ROS2-TMS for Construction is able to operate while verifying the current state of the OPERA-compatible construction machinery.


Thus, tms_if_for_opera is an indispensable module for operating OPERA-compatible construction machinery from the task management system of ROS2-TMS for Construction. In the future, we plan to add interface functionalities to the tms_if_for_opera module for OPERA-compatible construction machinery, including not only the backhoe ZX200 but also crawler dumps such as the IC120 and other OPERA-compatible construction machinery.

``` terminal_1
ros2 launch zx200_bringup vehicle.launch.py 
```

``` terminal_2
ros2 launch tms_ts_launch tms_ts_construction.launch.py
```

``` terminal_3
ros2 launch tms_if_for_opera tms_if_for_opera.launch.py 
```
ボタンぽち
