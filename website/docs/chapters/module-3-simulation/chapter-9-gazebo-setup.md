---
title: "Chapter 9 - Gazebo Simulation Setup"
module: "Digital Twin Simulation"
chapter: 9
description: "Gazebo simulation setup for humanoid robot development"
learningObjectives:
  - "Install and configure Gazebo for humanoid simulation"
  - "Create simulation environments for humanoid robots"
  - "Integrate with ROS 2 for control and perception"
prerequisites: ["chapter-8-ros2-tools"]
difficulty: "intermediate"
---

## Learning Objectives

- Install and configure Gazebo for humanoid simulation
- Create simulation environments for humanoid robots
- Integrate with ROS 2 for control and perception

## Introduction

Gazebo provides a high-fidelity physics simulation environment essential for developing and testing humanoid robots before deployment to physical hardware. As part of our "Sim-to-Real Rigor" principle from the project constitution, Gazebo enables training of humanoid robots in virtual environments with realistic physics, sensor simulation, and environmental interactions. This chapter covers the setup and configuration of Gazebo for humanoid robot simulation, with special attention to the physics and sensor simulation requirements that support the Vision-Language-Action pipeline and enable safe testing of bipedal locomotion and dexterous manipulation capabilities.

## Gazebo Overview

Gazebo is a 3D simulation environment that provides:

- **Physics Engine**: Realistic simulation of rigid body dynamics
- **Sensor Simulation**: Cameras, LIDAR, IMUs, force/torque sensors
- **Environment Modeling**: Complex 3D worlds with realistic lighting
- **ROS Integration**: Seamless integration with ROS 2 for control and perception

### Key Components

- **Gazebo Classic**: The traditional Gazebo simulation environment
- **Ignition Gazebo**: The newer, more modular version (Garden and newer)
- **Gazebo GUI**: Visual interface for simulation monitoring
- **Gazebo Server**: Headless simulation backend

## Installing Gazebo

### Prerequisites

Before installing Gazebo, ensure ROS 2 is properly installed:

```bash
# Verify ROS 2 installation
source /opt/ros/humble/setup.bash  # Or your ROS 2 distribution
echo $ROS_DISTRO
```

### Installing Gazebo for ROS 2 Humble

```bash
# Install Gazebo with ROS 2 integration
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
sudo apt install gazebo
```

### Alternative: Installing Ignition Gazebo (Garden)

For the latest features and better performance:

```bash
# Add Ignition repository
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/gazebo-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo.list > /dev/null

sudo apt update
sudo apt install ignition-garden
```

## Basic Gazebo Configuration

### Environment Variables

Set up environment variables for Gazebo:

```bash
# Add to ~/.bashrc or ~/.zshrc
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/.gazebo/models:~/ros2_ws/src/my_robot_description/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/ros2_ws/src/my_robot_description/worlds
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/ros2_ws/install/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/ros2_ws/install/lib
```

### Gazebo World Files

Create a basic world file `worlds/humanoid_test.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_test">
    <!-- Include a default ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a default light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add some objects for testing -->
    <model name="table">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 0.8 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.2 1</ambient>
            <diffuse>0.8 0.6 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add a simple box for manipulation -->
    <model name="box">
      <pose>2.2 0.1 0.9 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
            <diffuse>0.2 0.8 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.001</iyy>
            <iyz>0</iyz>
            <izz>0.001</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

## Integrating Humanoid Robot with Gazebo

### URDF to SDF Conversion

Gazebo can work directly with URDF files through the libgazebo_ros_xacro plugin:

Create a launch file `launch/humanoid_gazebo.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Gazebo with custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'worlds',
                'humanoid_test.world'
            ])
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0', '-y', '0', '-z', '1.0'  # Start 1m above ground for testing
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command([
                PathJoinSubstitution([FindPackageShare('my_robot_description'), 'urdf', 'humanoid.urdf.xacro'])
            ])
        }]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

### Adding Gazebo Plugins to URDF

Enhance your humanoid URDF with Gazebo-specific plugins:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_gazebo">

  <!-- Include the basic robot definition -->
  <xacro:include filename="humanoid.urdf.xacro"/>

  <!-- Gazebo plugins for ROS control -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid_robot</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>

  <!-- Gazebo plugins for sensors -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera_sensor">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
        <image>
          <format>R8G8B8</format>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>10.0</max_depth>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU sensor for balance feedback -->
  <gazebo reference="torso">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>  <!-- High rate for balance control -->
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <topicName>imu/data</topicName>
        <bodyName>torso</bodyName>
        <updateRateHZ>100.0</updateRateHZ>
        <gaussianNoise>0.0</gaussianNoise>
        <frameName>torso</frameName>
        <initialOrientationAsReference>false</initialOrientationAsReference>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Force/Torque sensor in left foot for balance detection -->
  <gazebo reference="left_foot">
    <sensor name="left_foot_ft_sensor" type="force_torque">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <force_torque>
        <frame>child</frame>
        <measure_direction>child_to_parent</measure_direction>
      </force_torque>
      <plugin name="left_foot_ft_plugin" filename="libgazebo_ros_ft_sensor.so">
        <topicName>left_foot/force_torque</topicName>
        <frameName>left_foot</frameName>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Joint state publisher for Gazebo -->
  <gazebo>
    <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
      <update_rate>30</update_rate>
      <joint_name>left_hip_joint</joint_name>
      <joint_name>left_knee_joint</joint_name>
      <joint_name>left_ankle_joint</joint_name>
      <joint_name>right_hip_joint</joint_name>
      <joint_name>right_knee_joint</joint_name>
      <joint_name>right_ankle_joint</joint_name>
      <joint_name>left_shoulder_joint</joint_name>
      <joint_name>left_elbow_joint</joint_name>
    </plugin>
  </gazebo>

</robot>
```

## Physics Configuration for Humanoid Simulation

### Physics Engine Parameters

Configure physics parameters for realistic humanoid simulation in your world file:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- Smaller steps for stability -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
  <ode>
    <solver>
      <type>quick</type>
      <iters>100</iters>  <!-- More iterations for stability -->
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.000001</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Material Properties for Realistic Simulation

Add material properties for realistic contact simulation:

```xml
<material name="gazebo/Blue">
  <ambient>0.0 0.0 0.8 1.0</ambient>
  <diffuse>0.0 0.0 0.8 1.0</diffuse>
  <specular>0.0 0.0 0.8 1.0</specular>
</material>

<material name="gazebo/Red">
  <ambient>0.8 0.0 0.0 1.0</ambient>
  <diffuse>0.8 0.0 0.0 1.0</diffuse>
  <specular>0.8 0.0 0.0 1.0</specular>
</material>
```

## Control Integration with ROS 2

### ROS 2 Control Configuration

Create a control configuration file `config/humanoid_control.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    left_leg_controller:
      type: position_controllers/JointGroupPositionController

    right_leg_controller:
      type: position_controllers/JointGroupPositionController

    torso_controller:
      type: position_controllers/JointGroupPositionController

left_leg_controller:
  ros__parameters:
    joints:
      - left_hip_joint
      - left_knee_joint
      - left_ankle_joint

right_leg_controller:
  ros__parameters:
    joints:
      - right_hip_joint
      - right_knee_joint
      - right_ankle_joint

torso_controller:
  ros__parameters:
    joints:
      - neck_joint
      - left_shoulder_joint
      - left_elbow_joint
```

### Launching with Controllers

Update your launch file to include ROS 2 control:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

def generate_launch_description():
    # Launch Gazebo with custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'worlds',
                'humanoid_test.world'
            ])
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command([
                'xacro ',
                PathJoinSubstitution([FindPackageShare('my_robot_description'), 'urdf', 'humanoid_with_gazebo.urdf.xacro'])
            ])
        }]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen'
    )

    # Load controller configurations
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    left_leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_leg_controller", "-c", "/controller_manager"],
    )

    right_leg_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_leg_controller", "-c", "/controller_manager"],
    )

    torso_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["torso_controller", "-c", "/controller_manager"],
    )

    # Create launch description and add actions
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[left_leg_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=left_leg_controller_spawner,
                on_exit=[right_leg_controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=right_leg_controller_spawner,
                on_exit=[torso_controller_spawner],
            )
        ),
    ])
```

## Vision System Integration

### Camera Configuration for VLA Pipeline

For the Vision-Language-Action pipeline, configure cameras with appropriate parameters:

```xml
<gazebo reference="camera_link">
  <sensor type="depth" name="rgbd_camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
      <image>
        <format>R8G8B8</format>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
      <min_depth>0.1</min_depth>
      <max_depth>10.0</max_depth>
      <update_rate>30.0</update_rate>
      <hack_baseline>0.07</hack_baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
    </plugin>
  </sensor>
</gazebo>
```

## Simulation Optimization for Humanoid Robots

### Performance Considerations

For humanoid robots with many degrees of freedom:

1. **Reduce Update Rates**: Use appropriate update rates for different sensors
2. **Simplify Collision Geometry**: Use simpler shapes for collision detection
3. **Limit Physics Steps**: Balance accuracy with performance

### Stability Tips for Bipedal Simulation

1. **Proper Mass Distribution**: Ensure realistic inertial properties
2. **Appropriate Joint Limits**: Prevent impossible configurations
3. **Sufficient Damping**: Add damping to prevent oscillations
4. **Small Time Steps**: Use smaller physics steps for stability

## Constitution Alignment

This chapter addresses several constitutional requirements:

### Sim-to-Real Rigor (Principle III)
- High-fidelity physics simulation for realistic humanoid behavior
- Proper sensor simulation matching real hardware
- Physics parameters tuned for accurate real-world behavior

### Real-Time Validation (Principle IV)
- High update rates for IMU sensors (100Hz) for balance feedback
- Proper QoS profiles for real-time sensor data
- Performance optimization for real-time simulation

### Target Hardware Optimization
- Simulation parameters optimized for deployment to Jetson Orin Nano
- Efficient sensor simulation within computational constraints

## Practical Examples

### Example 1: Humanoid Balance Testing Environment

Create a specialized world for balance testing:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="balance_test">
    <!-- Physics configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Ground plane with texture -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Balance testing features -->
    <model name="narrow_beam">
      <pose>2 0 0.05 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2 0.1 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 0.1 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Sloped surface for testing -->
    <model name="slope">
      <pose>-2 0 0 0 0.2 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 2 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 2 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.7 1</ambient>
            <diffuse>0.3 0.3 0.7 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Example 2: ROS 2 Node for Simulation Control

Create a node to test humanoid control in simulation:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

class SimulationController(Node):
    def __init__(self):
        super().__init__('simulation_controller')

        # QoS profile for simulation (can be best effort)
        sim_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=HistoryPolicy.VOLATILE
        )

        # Subscriptions
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            sim_qos
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            sim_qos
        )

        # Publishers
        self.left_leg_pub = self.create_publisher(
            Float64MultiArray,
            '/left_leg_controller/commands',
            10
        )

        self.right_leg_pub = self.create_publisher(
            Float64MultiArray,
            '/right_leg_controller/commands',
            10
        )

        # Balance control timer
        self.control_timer = self.create_timer(0.01, self.balance_control)  # 100Hz

        # State variables
        self.current_orientation = None
        self.current_joint_positions = {}

        self.get_logger().info('Simulation controller initialized')

    def imu_callback(self, msg):
        """Process IMU data for balance feedback"""
        self.current_orientation = msg.orientation
        # Extract roll, pitch, yaw for balance control
        # Simplified - use proper quaternion to RPY conversion in practice

    def joint_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def balance_control(self):
        """Simple balance control algorithm"""
        if self.current_orientation is None:
            return

        # Simplified balance control - in practice, use proper control theory
        # Extract pitch from orientation (simplified)
        pitch = 2 * (self.current_orientation.z * self.current_orientation.w -
                     self.current_orientation.x * self.current_orientation.y)

        # Calculate correction based on pitch error
        correction = -pitch * 10.0  # Proportional control

        # Send commands to legs to maintain balance
        left_cmd = Float64MultiArray()
        left_cmd.data = [0.0, 0.0, correction]  # hip, knee, ankle

        right_cmd = Float64MultiArray()
        right_cmd.data = [0.0, 0.0, correction]  # hip, knee, ankle

        self.left_leg_pub.publish(left_cmd)
        self.right_leg_pub.publish(right_cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = SimulationController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down simulation controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercises

### Exercise 1: Gazebo Environment Creation
Create a Gazebo world file that includes:
- A humanoid-sized room with furniture
- Multiple surfaces with different friction properties
- Obstacles for navigation testing
- Proper lighting for camera simulation

### Exercise 2: Sensor Integration
Enhance your humanoid URDF with:
- RGB-D camera for vision processing
- IMU sensors for balance feedback
- Force/torque sensors in feet
- Proper Gazebo plugins for each sensor

### Exercise 3: Control System Integration
Implement a complete simulation setup with:
- ROS 2 control configuration for all humanoid joints
- Proper launch files for starting simulation
- Balance control node for bipedal stability
- Performance optimization for real-time operation

## Summary

Gazebo provides the essential simulation environment for developing humanoid robots, enabling safe testing of complex behaviors before deployment to physical hardware. The high-fidelity physics and sensor simulation support the Vision-Language-Action pipeline and enable proper validation of bipedal locomotion and dexterous manipulation capabilities. Understanding Gazebo setup and configuration is crucial for effective humanoid robot development within our Sim-to-Real Rigor framework.

## Further Reading

- "Gazebo Tutorial" - Official Gazebo documentation
- "Programming Robots with ROS" by Quigley et al. (Simulation chapter)
- "Mastering ROS for Robotics Programming" by Jayanam (Gazebo section)
- "Robotics, Vision and Control" by Peter Corke (Simulation chapter)
- "Gazebo and ROS Integration Guide" - ROS Wiki