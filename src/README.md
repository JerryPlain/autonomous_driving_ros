# 🚗📄 README — PID Controller Node for Unity Vehicle Simulation

## ✅ Overview

This repository includes a ROS-based PID controller node (`pid_controller_node`) designed to control a simulated vehicle in Unity.  
It uses closed-loop PID control to track waypoints published to `/move_base_simple/goal`, leveraging real-time feedback on vehicle position and speed.

---

## ✅ Features

- Subscribes to `/move_base_simple/goal` for target waypoints (`geometry_msgs::PoseStamped`)
- Subscribes to vehicle pose from `/Unity_ROS_message_Rx/OurCar/CoM/pose`
- Subscribes to vehicle speed from `/Unity_ROS_message_Rx/OurCar/CoM/twist`
- PID control for:
  - Throttle (longitudinal velocity)
  - Steering (yaw angle)
- Publishes control commands to the topic `car_command` (`simulation::VehicleControl`)

---

⚠️ Please note first: 
you need to change line 78 in the file /src/path_recorder/src/csv_goal_publisher.cpp
```bash
std::string csv_path = "/src/path_recorder/recorded_path.csv";
```
to the actual CSV file path on your computer!

## ✅ Build the Workspace

From the root of your workspace, run:

```bash
catkin build
```
Then, in a new terminal, launch the Unity simulation node:

```bash
source devel/setup.bash
roslaunch simulation simulation.launch
```

Start the 3 nodes of the traffic_light_detector package
  ```bash
    source devel/setup.bash
    roslaunch traffic_light_detector detection_pipeline.launch
  ```

Start the node of the decision making package
  ```bash
  source devel/setup.bash
  rosrun decision_making decision_making_node
  ```

✅ Running the Dummy Controller Node
If you'd like to test a simple dummy controller , run:

```bash
source devel/setup.bash
rosrun dummy_controller dummy_controller_node
```

✅ Running the CSV Goal Publisher
After making sure that both roscore and the simulation node are running, start the CSV goal publisher:

```bash
source devel/setup.bash
rosrun path_recorder csv_goal_publisher
```