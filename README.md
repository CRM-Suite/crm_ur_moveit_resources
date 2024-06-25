# crm_ur3_moveit_resources
Resources for configuring and using the Universal Robots arms with MoveIt for the Cooperative Robotic Manufacturing System (CRM). It includes configuration files, launch files, and other utilities for a single and dual arm setup to help get started with planning and executing movements.

## Features

- **Custom MoveIt Configuration:** Tailored setup for the the CRM system to quickly start the UR arms with MoveIt
- **Visualization:** Support for RViz visualization to help mirror and debug the robot's movements
- **Examples:** Scripts to demonstrate basic and advanced usage of the MoveIt API
  1. MoveIt C++ API Examples
  2. MoveItPy API Examples
  3. Jupyter Notebook Prototyping
  4. Realtime Servo Examples

## Installation

1. Clone this repository into your ROS workspace:
    ```bash
    git clone <repository-url> ~/catkin_ws/src
    ```
2. Build the workspace:
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```
3. Source the workspace:
    ```bash
    source ~/catkin_ws/devel/setup.bash
    ```

## Usage for Single Arm Setup

1. **Starting the Universal Robots ROS2 Driver:**
   Source the installation
   ```
   source install/setup.bash
   ```
   
   ```bash
    ros2 launch crm_control start_robot.launch.py
    ```
    Set the use_fake_hardware:=true if working in simulation.

3. **Running Example Applications:**
    ```bash
    ros2 launch crm_moveit_config ur_moveit.launch.py
    ```

## Configuration

- **URDF/Xacro Files:** Located in the `urdf` directory, these files define the robot's physical and kinematic properties.
- **SRDF Files:** Located in the `config` directory, these files contain the Semantic Robot Description Format for MoveIt.
- **YAML Configurations:** Various configuration settings for the robot and MoveIt can be found in the `config` directory.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Authors
- Vihaan Shah [@ShahhhVihaan](https://github.com/ShahhhVihaan)

> [!NOTE]  
> Highlights information that users should take into account, even when skimming.

> [!TIP]
> Optional information to help a user be more successful.

> [!IMPORTANT]  
> Crucial information necessary for users to succeed.

> [!WARNING]  
> Critical content demanding immediate user attention due to potential risks.

> [!CAUTION]
> Negative potential consequences of an action.
