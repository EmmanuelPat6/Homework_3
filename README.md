# HOMEWORK_3 PATELLARO EMMANUEL P38000239 #
# 👀📷 Implement a Vision-Based Task 📷👀 #
This README file will show the instructions on how to build and run the Homework_3 Project 

## Features 🪐 ##
- Computer Vision 💻👁️
- Creation and Detection of a Spherical Object in Gazebo 🏀🔍
- ArUco Tag 🏷️🧩
- Vision-Based Control both with Velocity and Effort Commands 🚀⚡
- Positionin Task 📍🎯
- Look at Point Task 👓🕵️

## Available Directory in this Repository 📂 ##
- kdl
- ros2_kdl_package
- ros2_iiwa
- ros2_vision

## Getting Started ⏯️
1. Follow the guide to install ROS2 in Docker [here](https://github.com/RoboticsLab2024/ros2_docker_scripts.git)
2. Clone this repo in your `src` folder inside the `ros2_ws`
    ```shell
    cd src
    git clone https://github.com/EmmanuelPat6/Homework_3.git
    ```
3. Build the packages and make them visible to your workspace
    ```shell
    cd ..
    colcon build
    source install/setup.bash
    ```
**NOTE**: To make life easier, a world in Gazebo with *zero gravity* has been used in order to compensate the gravity itself. For this reason, in the controller it is no more necessary to explicitly have the *Gravity Compensation Term* of the *PD+ Controller*.

⚠️⚠️⚠️

If the launch of the world in Gazebo gives some problems and errors, run
```shell
export GZ_SIM_RESOURCE_PATH=~/ros2_ws/src/Homework_3/ros2_iiwa/iiwa_description/gazebo/models
```
this because in the **Dockerfile** is specified only `src` but when you download my Repository, you download all in anoter directory called `Homework_3`

⚠️⚠️⚠️
## Implementation 💻
### Spherical Object Detection  

1. 🤖🤖 An instruction to spawn the robot in Gazebo inside the world containing the **Spherical Object**
    ```shell
    ros2 launch iiwa_bringup iiwa_sphere.launch.py use_sim:=true use_vision:=true
    ```
    `use_sim:=true` to spawn the robot in Gazebo and `use_vision:=true` to spawn the robot with a **Camera Sensor**

⚠️⚠️⚠️ It is **NECESSARY** to act very quickly by pressing the play button in the bottom left corner to ensure the controllers are activated. If this is not done, you will need to close Gazebo, reissue the same command, and repeat the steps. ⚠️⚠️⚠️

2. 📷🎥 An istruction to view the image send by the **Camera Sensor** to the manipulator  
    ```shell
    ros2 run rqt_image_view rqt_image_view
    ```
   selecting the topic `/videocamera`

3. 🪟🕵️‍♂️ An istruction to open a new window in which it is possible to see the **Detection** of the **Spherical Object** executed by the manipulator through the **Camera Sensor**
    ```shell
    ros2 run ros2_opencv ros2_opencv_node
    ```
    
### Vision-Based Control with Velocity Commands 🏎📷

1. 🤖🤖 An instruction to spawn the robot in Gazebo inside the world containing the **ArUco Tag 201** with a **Velocity Controller**
    ```shell
    ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:=true use_vision:=true
    ```
    `command_interface:="velocity"` and `robot_controller:="velocity_controller"` to spawn the robot with a **Velocity Interfare** and a **Velocity Controller**, `use_sim:=true` to spawn the robot in Gazebo and `use_vision:=true` to spawn the robot with a **Camera Sensor**

⚠️⚠️⚠️ It is **NECESSARY** to act very quickly by pressing the play button in the bottom left corner to ensure the controllers are activated. If this is not done, you will need to close Gazebo, reissue the same command, and repeat the steps. ⚠️⚠️⚠️

2. 📐📏 An istruction to allow the **ArUco Tag 201 Detection**  
    ```shell
    ros2 launch aruco_ros single.launch.py 
    ```
   
3. 🗺️📸 An istruction to see the environment with the **ArUco Detection** through the **Camera Sensor**
    ```shell
    ros2 run rqt_image_view rqt_image_view
    ```
    selecting the topic `/aruco_single/result`

4. 🚀📍An instruction to do the **Positioning Task** in order to align the **Camera** to the **ArUco Marker** with a desired **Position and Orientation Offsets**
   ```shell
    ros2 run ros2_kdl_package ros2_kdl_node_vision_control 
    ```
5. 👀🎯After that the **Positioning** is completed (wait the message `Positioning Task Executed Successfully ...`) it is possible to run in this last terminal, after pressing `ctrl+C`, the final instruction
   ```shell
    ros2 run ros2_kdl_package ros2_kdl_node_vision_control --ros-args -p task:=look-at-point
    ```
   which performs a **Look-at-Point Task** using a desired **Control Law** specified in the code and in the report. Now it is possible to move the ArUco Tag with the realtive interface and the center of the **Camera Sensor** will align with      the center of the Tag.

 ### Vision-Based Control with Effort Commands 🏎📷

1. 🤖🤖 An instruction to spawn the robot in Gazebo inside the world containing the **ArUco Tag 201** with a **Effort Controller**
    ```shell
    ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_sim:=true use_vision:=true
    ```
    `command_interface:="velocity"` and `robot_controller:="velocity_controller"` to spawn the robot with a **Velocity Interfare** and a **Velocity Controller**, `use_sim:=true` to spawn the robot in Gazebo and `use_vision:=true` to spawn the robot with a **Camera Sensor**

⚠️⚠️⚠️ It is **NECESSARY** to act very quickly by pressing the play button in the bottom left corner to ensure the controllers are activated. If this is not done, you will need to close Gazebo, reissue the same command, and repeat the steps. ⚠️⚠️⚠️

2. 📐📏 An istruction to allow the **ArUco Tag 201 Detection**  
    ```shell
    ros2 launch aruco_ros single.launch.py 
    ```
   
3. 🗺️📸 An istruction to see the environment with the **ArUco Detection** through the **Camera Sensor**
    ```shell
    ros2 run rqt_image_view rqt_image_view
    ```
    selecting the topic `/aruco_single/result`

4. 🚀📍An instruction to do the **Positioning Task** in order to align the **Camera** to the **ArUco Marker** with a desired **Position and Orientation Offsets**
   ```shell
    ros2 run ros2_kdl_package ros2_kdl_node_vision_effort_control --ros-args -p task:=positioning
    ```
5. 👀📏An instruction to do the **Look-at-Point Task** with a **Linear Trajectory**
   ```shell
    ros2 run ros2_kdl_package ros2_kdl_node_vision_effort_control --ros-args -p task:=look-at-point
    ```
   which, using a new **Orientation Error** given by `sd-s`, implement the **Linear Trajectory** implemented in the previous Homework but looking, during it, the **ArUco Tag**

To implement these last two points with the **Operational Space Inverse Dynamics Control** it is sufficient to add at teh end of each instruction `-p cmd_interface:=cart_effort`
