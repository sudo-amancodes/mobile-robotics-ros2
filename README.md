# How to Set Up and Run the F1Tenth Simulator and Code

This guide assumes you are running on Ubuntu (20.04), have ROS 2 installed, and F1Tenth Simulator.

## Prerequisites

1. ROS 2 (e.g., Foxy):  
   Follow the official ROS 2 installation instructions:

   - [https://docs.ros.org/en/foxy/Installation.html](https://docs.ros.org/en/foxy/Installation.html)

2. F1Tenth Simulator:  
   Follow the official F1Tenth Simulation instructions:
   - [https://github.com/f1tenth/f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros)

If errors occur in installing the Simulation. Run the command:

- `rosdep update --include-eol-distros`
- `rosdep install -i --from-path src --rosdistro foxy -y`

## Start Simulator

1. After you are able to run you own simulator without problems. Install my sim and run that. My simulator contain map data that allows you to run the Circuit of Americas map.
2. In a new terminal, run the following command in my simulator directory.
   ```
   source /opt/ros/foxy/setup.bash
   source install/local_setup.bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```

## Run Algorithms

1. In different terminal, access and move into `ros2_ws`. Once here colcon build.
   ```
   cd ros2_ws
   source /opt/ros/foxy/setup.bash
   colcon build
   ```
2. Run one of the Python algorithm (Pure Pursuit, Wall Following, Follow-The-Gap). Below are all three algorithms in order (run one at a time):

`ros2 run wall_follow wall_follow_node.py`

`ros2 run pure_pursuit pure_pursuit_node.py`

`ros2 run gap_follow reactive_node.py`

For Pure Pursuit, if it does not work, make sure you have the correct path variable for actual `map_absolute_path` variable. Since it was based on my actual path.

Make sure you have both terminals running and you should see the algorithms working in RVIZ.

## Code Structure and Algorithms

This project implements three autonomous racing algorithms for the F1TENTH simulator:

1. **Follow the Gap (gap_follow)**: A reactive obstacle avoidance algorithm that processes LiDAR scans to find the largest gap between obstacles and steers toward it. The implementation includes preprocessing LiDAR data, finding the closest point, creating a "bubble" around obstacles, identifying the maximum gap, and selecting the best steering angle.

2. **Pure Pursuit (pure_pursuit)**: A path tracking algorithm that follows predefined waypoints. It works by calculating the geometric path to the next waypoint, transforming coordinates to the vehicle's frame of reference, and computing steering angles to maintain the vehicle on the racing line.

3. **Wall Following (wall_follow)**: An algorithm that maintains a constant distance from walls by using LiDAR readings to detect walls and adjust steering accordingly, allowing the vehicle to navigate tracks efficiently.

Each algorithm is implemented as both a C++ class and a Python node, providing flexibility for development and testing. The code structure follows ROS 2 conventions with proper publisher/subscriber patterns for vehicle control.

## Acknowledgements

Special thanks to [RoboRacer.ai](https://roboracer.ai/learn) for making the scaffolding code, labs, simulation workspace, and ros2 f1tenth gym environments available, which provided the foundation for implementing these autonomous racing algorithms.

We also acknowledge the contributors who provided public access to race track data, enabling realistic testing and evaluation of the autonomous racing algorithms. In particular (https://github.com/f1tenth/f1tenth_racetracks):

Betz, J., Zheng, H., Liniger, A., Rosolia, U., Karle, P., Behl, M., Krovi, V., & Mangharam, R. (2022). Autonomous vehicles on the edge: A survey on autonomous vehicle racing. IEEE Open Journal of Intelligent Transportation Systems, 3, 458-488. https://doi.org/10.1109/ojits.2022.3181510
