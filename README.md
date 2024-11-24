# **Walker Simulation with ROS 2 and Gazebo**

This project demonstrates the implementation of a walker robot using ROS 2 (Humble) and Gazebo. The robot, based on the TurtleBot3 Burger, alternates between moving forward and turning when an obstacle is detected. The simulation is integrated with rosbag for recording data.

---

## **Features**

- **Obstacle Detection:** Uses LIDAR (`/scan` topic) to detect obstacles.
- **State Design Pattern:** Alternates between moving and turning states based on sensor data.
- **Bag Recording:** Records simulation data for inspection and playback.
- **Replay Capability:** Inspect and replay rosbag files to review robot behavior.

---

## **Dependencies**

The following ROS 2 packages are required:

1. **`rclcpp`**
   - ROS 2 C++ client library for building nodes.
2. **`geometry_msgs`**
   - Messages for controlling robot motion via velocity commands.
3. **`sensor_msgs`**
   - Messages for processing LIDAR data from the `/scan` topic.

### **Installing Dependencies**

Ensure the required ROS 2 packages are installed:

```bash
sudo apt-get update
sudo apt-get install ros-humble-rclcpp ros-humble-geometry-msgs ros-humble-sensor-msgs
```
## Assumptions/Environment
- ROS 2 Humble installed on Ubuntu Jammy (22.04).

- TurtleBot3 Burger robot model is used.
- Gazebo 11 simulator.
- Workspace structured as follows:
```bash
~/walker_ws
└── src
    └── my_gazebo_tutorials
        ├── CMakeLists.txt
        ├── include
        ├── launch
        │   ├── replay_world.launch.py
        │   └── walker_bot.launch.py
        ├── package.xml
        ├── README.md
        ├── results
        │   ├── clang_tidy.txt
        │   ├── cpplint.txt
        │   └── walker_bag
        │       ├── metadata.yaml
        │       └── walker_bag_0.db3
        ├── src
        │   └── walker_class.cpp
        └── worlds
            ├── newest_1
            ├── newest_1.world
            └── newest_2.world

8 directories, 13 files
```

## Setup Instructions
### Build the Package
Clone the repository:
```bash

cd ~/walker_ws/src
git clone https://github.com/Prathinav-kV/my_gazebo_tutorials.git
```
## IMPORTANT

In a VScode please open the folder "my_gazebo_tutorials" which contains the 'walker' package:
```bash
cd ~/walker/src/my_gazebo_tutorials
code .
```
Navigate to the launch/walker_bot.launch.py file:
Please change the following line within the code to suit your system:
- Line 20: '/home/pratkv/walker_ws/src/my_gazebo_tutorials/worlds/newest_2.world'
- Change this according to the name of your user from 'pratkv' to your user_name

Similarly for the file launch/replay_world.launch.py file:
- Line 11: '/home/pratkv/walker_ws/src/my_gazebo_tutorials/worlds/newest_2.world'
- Change this according to the name of your user from 'pratkv' to your user_name

### Navigate to the workspace and build the package:
```bash
cd ~/walker_ws
colcon build 
source ~/walker_ws/install/setup.bash
```

## Running the Simulation

To run the simulation and test the walker functionality, follow these steps:

1. **Launch the Simulation with Bag Recording Enabled**  
   Run the following command to start the simulation and enable bag recording:
   ```bash
   ros2 launch walker walker_bot.launch.py
   ```
   - This will launch Gazebo with the custom world newest_2.world, spawn the TurtleBot3 Burger, and start the walker node.
   - Let it run for 15-30 seconds and then terminate the simulation using Ctrl+C
2. Bag recording will automatically begin, and the recorded data will be stored in:
    ```bash
    ~/walker_ws/results/walker_bag
    ```
3. (OPTIONAL) Run the Simulation with Bag Recording Disabled
If you want to disable bag recording, use the following command:

    ```bash
    ros2 launch walker walker_bot.launch.py enable_bag_recording:=false
    ```
    This launches the simulation and walker node without recording data.

4. Verify the Bag File

    After stopping the simulation, inspect the bag file to ensure the data was recorded correctly.
    Use the following command to check the contents of the bag file:
    ```bash
    ros2 bag info ~/walker_ws/results/walker_bag
    ```
    Example output:
    ```yaml
    Files: walker_bag_0.db3
    Bag size: 5.8 MiB
    Storage id: sqlite3
    Duration: 30s
    Messages: 12345
    Topic information:
        Topic: /cmd_vel (1234 messages)
        Topic: /scan (5678 messages)
    ```
5. Replay the Bag File

    Launch the simulation without the walker node or bag recording by running the following command:
    ```bash
    ros2 launch walker replay_world.launch.py
    ```
    Play back the recorded bag file to verify the recorded data in a new terminal:
    ```bash
    ros2 bag play ~/walker_ws/results/walker_bag
    ```
6. Inspect Topics During Playback

    Verify the topics being replayed in a new terminal:
    ```bash
    ros2 topic list
    ```
    Echo the /cmd_vel topic to see the velocity commands:
    ```bash
    ros2 topic echo /cmd_vel
    ```
    Example output:
    ```yaml
    linear:
    x: 0.2
    angular:
    z: 0.0
    ```

### Conclusion

The **walker** package demonstrates the implementation of a state-based autonomous robot navigating in a simulated Gazebo environment using ROS 2. By leveraging sensor data, the TurtleBot3 Burger can dynamically detect obstacles and alternate between moving and turning states to avoid collisions. 

The project incorporates essential ROS 2 features, such as node communication, laser scan data processing, and velocity control, while utilizing Gazebo for accurate simulation. The bag recording functionality ensures that simulation data can be logged, analyzed, and replayed for further validation and debugging.

Key takeaways from this project:
- Proper setup of environment variables and dependencies is crucial for smooth simulation and package functionality.
- ROS 2 bag recording and playback are powerful tools for debugging and testing robotic behaviors.
- Modular and reusable design, such as the use of state patterns, makes the system adaptable for future enhancements.

This project provides a solid foundation for further research and development in autonomous navigation and serves as a practical example of ROS 2 and Gazebo integration.
