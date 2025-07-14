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
A Unity simulation window should pop up, and you should be able to manually control the vehicle using the arrow keys or WASD.

💬 Note: This is a minimal example launch file to verify that everything in the Unity simulation is accessible via ROS. You are encouraged to create and customize your own launch files if needed.

✅ Running the Dummy Controller Node (Optional)
If you'd like to test a simple dummy controller (without PID), run:

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