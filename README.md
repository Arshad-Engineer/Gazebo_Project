# Gazebo_Project
# beginner_tutorials
# Overview:
 - Implement a simple walker algorithm much like a Roomba robot vacuum cleaner.
 - Google Style Guide was followed in all the cpp files
 - CPPCHECK and CPPLINT was run to perform static code analysis
 - 'results' folder contains the output of CPPCHECK and CPPLINT in a text file, bag file

## Author:
 - Arshad Shaik
    - UID: 118438832

## Dependencies/Requirements: 
 - Ubuntu 20.04 
 - VS Code/Terminal
 - ROS 2 galactic
 - Gazebo 

## Build & Run Instructions:
 - open terminal and source ros2
 ```
 source /opt/ros/galactic/setup.bash
 printenv | grep -i ROS
 ```

 - create a workspace (if not created before)
 ```
 mkdir -p ~/ros2_ws/src
 cd ~/ros2_ws/src/
 ```

 - Navigate to root of workspace
 ```
 cd <ros2_workspace_name>
 ```
   - or
 ```
 cd ros2_ws
 ```

 - Check for missing dependencies before building the package
 ```
 rosdep install -i --from-path src --rosdistro galactic -y
 ```

 - Build the package
 ```
 colcon build --packages-select gazebo_walker
 ```

 - open new terminal
 - Navigate to root of workspace
 ```
 cd ros2_ws
 ```
   - or
 ```
 cd <ros2_workspace_name>
 ```

 - source /opt/ros/galactic/setup.bash
 ```
 . install/setup.bash
 ```

 - run launch file
 ```
 ros2 launch gazebo_walker launch_walker.py record_flag:=True
 ```

## Command to run static code analysis:
 - Navigate to src folder in package
 ```
 cd ros2_ws/src/gazebo_walker/src
 ```
 - run the following command from src folder in package
 ```
 cppcheck --enable=all --std=c++17 *.cpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression > ./../results/cppcheckreport
 ```

## Command to check Google Style:
 - Navigate to src folder in package
 ```
 cd ros2_ws/src/gazebo_walker/src
 ```
 - run the following command from src folder in package
 ```
 cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order gazeboWalker.cpp > ./../results/cpplintreport
 ```
 - For reference puposes - clang formating command is listed here:
 ```
 clang-format -style=Google -i gazeboWalker.cpp
 ```
## Dependency Installation: 
- ROS 2 Galactic:
- Follow the below website instructions to install ROS 2 Galacticbased on your Ubuntu version
  - https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
- Gazebo
```
sudo apt install ros-galactic-gazebo-ros-pkgs
sudo apt install ros-galactic-turtlebot3*
sudo apt install ros-galactic-gazebo-plugins
```
- cpplint
```
sudo pip install cpplint
```
## Issues faced:
- There are lot of issues faced, initially with Gazebo in ROS2 Humble, then switched to galactic
