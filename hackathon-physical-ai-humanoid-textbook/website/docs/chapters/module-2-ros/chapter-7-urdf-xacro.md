---
title: "Chapter 7 - URDF & XACRO for Humanoid Robots"
module: "ROS 2: The Robotic Nervous System"
chapter: 7
description: "Explaining URDF and XACRO for modeling humanoid robots"
learningObjectives:
  - "Create URDF models for humanoid robots"
  - "Use XACRO for complex robot descriptions"
  - "Implement anthropomorphic design principles"
prerequisites: ["chapter-6-ros2-packages"]
difficulty: "intermediate"
---

## Learning Objectives

- Create URDF models for humanoid robots
- Use XACRO for complex robot descriptions
- Implement anthropomorphic design principles

## Introduction

Unified Robot Description Format (URDF) and its XML macro extension (XACRO) are fundamental tools for describing robot kinematics, dynamics, and visual properties in ROS 2. For humanoid robots operating in human-centered environments, proper URDF modeling is essential to capture the unique challenges of bipedal locomotion and dexterous manipulation as mandated by our project constitution's Anthropomorphic Focus principle. This chapter covers the creation of sophisticated URDF models for humanoid robots, with special attention to the anthropomorphic design requirements and the complex kinematic structures necessary for human-like movement.

## Understanding URDF

URDF (Unified Robot Description Format) is an XML format for representing a robot model:

- **Kinematics**: Joint and link relationships defining robot structure
- **Dynamics**: Mass, inertia, and friction properties
- **Visual**: Appearance for simulation and visualization
- **Collision**: Collision geometry for physics simulation

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_humanoid_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <joint name="base_to_head" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.7"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Anthropomorphic Design Principles

Humanoid robots must address the unique challenges of human-like form:

### Bipedal Locomotion Requirements
- **Center of Mass**: Position for stable walking
- **Foot Design**: Ground contact for balance
- **Leg Structure**: Joints for walking, running, climbing stairs

### Dexterous Manipulation Requirements
- **Hand Design**: Multiple degrees of freedom for grasping
- **Arm Structure**: Reach and dexterity for human tasks
- **Wrist Configuration**: Orientation for tool use

### Human-Centered Environment Adaptation
- **Height and Reach**: Compatible with human furniture and tools
- **Shoulder Configuration**: Human-like range of motion
- **Hip Design**: Bipedal stability with human-like movement

## Advanced URDF Concepts

### Joint Types for Humanoid Robots

```xml
<!-- Revolute joint (rotational with limits) -->
<joint name="hip_pitch" type="revolute">
  <parent link="torso"/>
  <child link="thigh"/>
  <origin xyz="0 0 -0.1" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0"/>
  <dynamics damping="1.0" friction="0.1"/>
</joint>

<!-- Continuous joint (unlimited rotation) -->
<joint name="head_pan" type="continuous">
  <parent link="neck"/>
  <child link="head"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <dynamics damping="0.5"/>
</joint>

<!-- Fixed joint (no movement) -->
<joint name="sensor_mount" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0" rpy="0 0 0"/>
</joint>
```

### Inertial Properties

Proper inertial properties are crucial for realistic simulation:

```xml
<inertial>
  <!-- Mass in kilograms -->
  <mass value="2.5"/>
  <!-- Inertia tensor - important for dynamic simulation -->
  <inertia
    ixx="0.01" ixy="0.0" ixz="0.0"
    iyy="0.01" iyz="0.0"
    izz="0.01"/>
</inertial>
```

### Visual and Collision Properties

```xml
<visual>
  <!-- Visual appearance for RViz and simulation -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <mesh filename="package://my_robot_description/meshes/upper_arm.dae"/>
  </geometry>
  <material name="gray">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
</visual>

<collision>
  <!-- Collision geometry for physics simulation -->
  <!-- Often simplified compared to visual geometry for performance -->
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <cylinder length="0.3" radius="0.05"/>
  </geometry>
</collision>
```

## XACRO: XML Macros for Robot Description

XACRO extends URDF with macros, properties, and mathematical expressions, making complex humanoid models more manageable:

### Basic XACRO Structure

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Properties for easy parameterization -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="robot_height" value="1.6" />
  <xacro:property name="link_radius" value="0.05" />
  <xacro:property name="link_length" value="0.3" />

  <!-- Macros for repeated structures -->
  <xacro:macro name="simple_link" params="name xyz_length">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder length="${xyz_length}" radius="${link_radius}"/>
        </geometry>
        <origin xyz="0 0 ${xyz_length/2}" rpy="0 0 0"/>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${xyz_length}" radius="${link_radius}"/>
        </geometry>
        <origin xyz="0 0 ${xyz_length/2}" rpy="0 0 0"/>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <inertia
          ixx="0.01" ixy="0" ixz="0"
          iyy="0.01" iyz="0"
          izz="0.01"/>
      </inertial>
    </link>
  </xacro:macro>

</robot>
```

## Complete Humanoid Robot URDF Example

Here's a more complete example of a humanoid robot using XACRO:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_humanoid">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="robot_height" value="1.6" />
  <xacro:property name="torso_height" value="0.6" />
  <xacro:property name="upper_leg_length" value="0.4" />
  <xacro:property name="lower_leg_length" value="0.4" />
  <xacro:property name="upper_arm_length" value="0.3" />
  <xacro:property name="lower_arm_length" value="0.3" />

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/4}" upper="${M_PI/4}" effort="10.0" velocity="3.0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0.1 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="20.0" velocity="3.0"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="15.0" velocity="3.0"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.05 -0.08 -0.25" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="50.0" velocity="3.0"/>
  </joint>

  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="${M_PI/2}" effort="40.0" velocity="3.0"/>
  </joint>

  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="4.0"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.04"/>
    </inertial>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/6}" upper="${M_PI/6}" effort="20.0" velocity="3.0"/>
  </joint>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right side (similar to left, mirrored) -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="0.05 0.08 -0.25" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="50.0" velocity="3.0"/>
  </joint>

  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="${M_PI/2}" effort="40.0" velocity="3.0"/>
  </joint>

  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="4.0"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.04"/>
    </inertial>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-M_PI/6}" upper="${M_PI/6}" effort="20.0" velocity="3.0"/>
  </joint>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Transmission for ROS Control -->
  <transmission name="left_hip_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_hip_joint">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_hip_motor">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <!-- Gazebo-specific elements -->
  <gazebo reference="base_link">
    <material>Gazebo/Orange</material>
  </gazebo>

</robot>
```

## Advanced XACRO Features for Humanoid Robots

### Mathematical Expressions

XACRO supports mathematical expressions for parameter calculations:

```xml
<xacro:property name="hip_offset_x" value="0.05" />
<xacro:property name="hip_offset_y" value="0.08" />
<xacro:property name="leg_length" value="0.8" />

<!-- Calculate derived values -->
<xacro:property name="foot_z" value="${-leg_length/2}" />
```

### Conditional Statements

```xml
<xacro:macro name="optional_sensor" params="install_sensor:=false">
  <xacro:if value="${install_sensor}">
    <joint name="sensor_joint" type="fixed">
      <parent link="head"/>
      <child link="camera_link"/>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
    </joint>
    <link name="camera_link"/>
  </xacro:if>
</xacro:macro>
```

### Include Other Files

```xml
<!-- Include common definitions -->
<xacro:include filename="$(find my_robot_description)/urdf/materials.xacro" />
<xacro:include filename="$(find my_robot_description)/urdf/common_properties.xacro" />
<xacro:include filename="$(find my_robot_description)/urdf/hand.xacro" />
```

## Vision-Language-Action Integration

URDF models support the VLA pipeline by providing:

### Kinematic Information
- **Forward Kinematics**: Calculate end-effector positions from joint angles
- **Inverse Kinematics**: Calculate joint angles for desired end-effector positions
- **Collision Checking**: Prevent self-collisions during manipulation

### Simulation Integration
- **Physics Properties**: Mass, inertia for realistic simulation
- **Sensor Mounting**: Proper placement of cameras, IMUs, and other sensors
- **Actuator Models**: Realistic joint dynamics for control development

## Testing and Validation

### URDF Validation

```bash
# Check URDF syntax
check_urdf /path/to/robot.urdf

# Parse XACRO to URDF
xacro input.xacro > output.urdf
```

### Visualization in RViz

```xml
<!-- Add TF publishing for visualization -->
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
```

### Simulation in Gazebo

```xml
<!-- Gazebo-specific elements -->
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/my_robot</robotNamespace>
  </plugin>
</gazebo>
```

## Constitution Alignment

This chapter addresses several constitutional requirements:

### Anthropomorphic Focus (Principle II)
- Emphasis on bipedal locomotion kinematics
- Dexterous manipulation structures
- Human-centered environment compatibility

### Sim-to-Real Rigor (Principle III)
- Accurate physical properties for simulation
- Proper sensor integration for perception
- Realistic dynamics for control development

### Visualization Requirements (Key Standard II)
- Use of proper geometric representations
- Appropriate materials and colors
- Clear kinematic structure visualization

## Practical Examples

### Example 1: Hand Model with XACRO

```xml
<xacro:macro name="robotic_hand" params="side parent_link position">
  <!-- Thumb -->
  <joint name="${side}_thumb_joint" type="revolute">
    <parent link="${parent_link}"/>
    <child link="${side}_thumb_knuckle"/>
    <origin xyz="${position}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="5.0" velocity="2.0"/>
  </joint>

  <link name="${side}_thumb_knuckle">
    <visual>
      <geometry>
        <cylinder length="0.04" radius="0.015"/>
      </geometry>
      <origin xyz="0 0 0.02" rpy="0 0 0"/>
    </visual>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Additional finger joints would continue similarly -->
</xacro:macro>
```

### Example 2: Complete Robot with Sensor Integration

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_sensors">

  <!-- Include basic structure -->
  <xacro:include filename="humanoid_base.xacro"/>

  <!-- RGB-D Camera -->
  <joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
    </visual>
  </link>

  <!-- IMU for balance feedback -->
  <joint name="imu_joint" type="fixed">
    <parent link="torso"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="imu_link"/>

  <!-- Gazebo plugins for sensors -->
  <gazebo reference="camera_link">
    <sensor type="depth" name="camera_sensor">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov>
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
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

## Exercises

### Exercise 1: Custom Humanoid Model
Create a URDF/XACRO model for a humanoid robot that includes:
- Complete kinematic chain for bipedal locomotion
- Anthropomorphic arms with 7 DOF each
- Proper inertial properties for dynamic simulation
- Mounting points for RGB-D camera and IMU

### Exercise 2: Modular Design
Design a modular URDF system using XACRO includes for:
- Reusable leg macro
- Reusable arm macro
- Reusable hand/foot macro
- Main robot assembly that combines these modules

### Exercise 3: Sensor Integration
Extend your robot model to include proper sensor integration for the VLA pipeline:
- Vision sensors (cameras) for perception
- IMU for balance feedback
- Force/torque sensors in joints
- Proper Gazebo plugins for simulation

## Summary

URDF and XACRO are essential tools for modeling humanoid robots, enabling the representation of complex kinematic structures necessary for bipedal locomotion and dexterous manipulation. The anthropomorphic focus of our approach is supported through proper modeling of human-like kinematics and dynamics. Understanding these tools is crucial for developing effective Physical AI systems that can operate in human-centered environments, with accurate models for both simulation and real-world deployment.

## Further Reading

- "Programming Robots with ROS" by Quigley et al. (URDF chapter)
- "Mastering ROS for Robotics Programming" by Jayanam
- "Robotics, Vision and Control" by Peter Corke
- "URDF for Dummies" - ROS Wiki tutorial
- "XACRO tutorial" - ROS Wiki