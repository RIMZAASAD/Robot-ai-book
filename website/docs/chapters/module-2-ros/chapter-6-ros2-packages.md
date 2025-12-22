---
title: "Chapter 6 - Creating ROS 2 Packages (Python)"
module: "ROS 2: The Robotic Nervous System"
chapter: 6
description: "Step-by-step instructions for creating ROS 2 packages in Python"
learningObjectives:
  - "Create ROS 2 packages using Python"
  - "Implement nodes with proper build system configurations"
  - "Configure QoS profiles for real-time applications"
prerequisites: ["chapter-5-ros2-architecture"]
difficulty: "intermediate"
---

## Learning Objectives

- Create ROS 2 packages using Python
- Implement nodes with proper build system configurations
- Configure QoS profiles for real-time applications

## Introduction

Creating ROS 2 packages is fundamental to building modular, reusable robotics software. For Physical AI systems that must operate in real-time with strict timing constraints, proper package structure and configuration are essential. This chapter provides step-by-step instructions for creating ROS 2 packages in Python, with special attention to the production-ready requirements specified in our project constitution, including proper build system configurations (package.xml, setup.py) necessary for deployment on target hardware like the NVIDIA Jetson Orin Nano.

## ROS 2 Package Structure

A standard ROS 2 package follows this directory structure:

```
my_robot_package/
├── CMakeLists.txt          # Build configuration for C++
├── package.xml             # Package metadata and dependencies
├── setup.py                # Python package configuration
├── setup.cfg               # Installation configuration
├── resource/               # Resource files
│   └── my_robot_package    # Executable resource links
├── my_robot_package/       # Python source code
│   ├── __init__.py
│   ├── my_node.py
│   └── my_module.py
└── test/                   # Test files
    └── test_my_node.py
```

## Creating a Python Package with colcon

### Step 1: Create the Package Directory

```bash
mkdir -p ~/ros2_ws/src/my_robot_package
cd ~/ros2_ws/src/my_robot_package
```

### Step 2: Create package.xml

The `package.xml` file contains metadata about your package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <description>Example package for humanoid robot control</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Step 3: Create setup.py

The `setup.py` file configures your Python package:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='Example package for humanoid robot control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_node = my_robot_package.my_robot_node:main',
            'balance_controller = my_robot_package.balance_controller:main',
            'vision_processor = my_robot_package.vision_processor:main',
        ],
    },
)
```

### Step 4: Create setup.cfg

```ini
[develop]
script-dir=$base/lib/my_robot_package
[install]
install-scripts=$base/lib/my_robot_package
```

## Creating Your First ROS 2 Node

### Step 5: Create the Python Package Directory

```bash
mkdir my_robot_package
touch my_robot_package/__init__.py
```

### Step 6: Create a Basic Node

Create `my_robot_package/my_robot_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')

        # Create a publisher for robot status
        self.status_publisher = self.create_publisher(
            String,
            'robot_status',
            10
        )

        # Create a subscriber for joint commands
        # Using QoS profile appropriate for control
        control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.joint_subscriber = self.create_subscription(
            JointState,
            'joint_commands',
            self.joint_callback,
            control_qos
        )

        # Timer for periodic status updates
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

        self.get_logger().info('MyRobotNode has been started')

    def joint_callback(self, msg):
        self.get_logger().info(f'Received joint positions: {msg.position}')
        # Process joint commands here

    def timer_callback(self):
        msg = String()
        msg.data = f'Robot status: Running for {self.i} seconds'
        self.status_publisher.publish(msg)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    my_robot_node = MyRobotNode()

    try:
        rclpy.spin(my_robot_node)
    except KeyboardInterrupt:
        pass
    finally:
        my_robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Production-Ready Node with Real-Time Considerations

For humanoid robots requiring real-time performance, nodes must be designed with timing constraints in mind. Here's an example of a balance controller node:

Create `my_robot_package/balance_controller.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration
import numpy as np
from collections import deque

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Configuration parameters
        self.declare_parameter('control_frequency', 100)  # 100Hz for balance
        self.declare_parameter('max_torque', 100.0)
        self.declare_parameter('balance_kp', 10.0)  # Proportional gain
        self.declare_parameter('balance_kd', 1.0)   # Derivative gain

        self.control_frequency = self.get_parameter('control_frequency').value
        self.max_torque = self.get_parameter('max_torque').value
        self.kp = self.get_parameter('balance_kp').value
        self.kd = self.get_parameter('balance_kd').value

        # Critical QoS for balance feedback
        balance_qos = QoSProfile(
            depth=1,  # Only most recent value matters
            reliability=ReliabilityPolicy.RELIABLE,  # Must not lose balance data
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            lifespan=Duration(seconds=0.01)  # 10ms lifespan for balance data
        )

        # Publishers and subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            balance_qos
        )

        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            1
        )

        # Store previous values for derivative calculation
        self.prev_pitch = 0.0
        self.prev_time = self.get_clock().now()

        # Initialize timing for control loop
        control_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(
            control_period,
            self.balance_control_loop
        )

        self.get_logger().info(f'Balance controller started at {self.control_frequency}Hz')

    def imu_callback(self, msg):
        """Process IMU data for balance feedback"""
        # Extract pitch angle from IMU orientation
        # This is simplified - in practice, you'd use proper quaternion math
        orientation = msg.orientation
        pitch = self.quaternion_to_pitch(orientation)

        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        # Store values for derivative calculation
        self.current_pitch = pitch
        self.dt = dt
        self.prev_time = current_time
        self.prev_pitch = pitch

    def quaternion_to_pitch(self, q):
        """Convert quaternion to pitch angle"""
        # Simplified conversion - use proper math in production
        sinr_cosp = 2 * (q.w * q.y - q.z * q.x)
        cosr_cosp = 1 - 2 * (q.y * q.y + q.x * q.x)
        pitch = np.arctan2(sinr_cosp, cosr_cosp)
        return pitch

    def balance_control_loop(self):
        """Main balance control loop running at configured frequency"""
        try:
            # Calculate pitch error (assuming desired pitch is 0)
            pitch_error = -self.current_pitch  # Negative to correct tilt

            # Calculate derivative for PD control
            if hasattr(self, 'dt') and self.dt > 0:
                pitch_rate = (self.current_pitch - self.prev_pitch) / self.dt
            else:
                pitch_rate = 0.0

            # PD control law
            control_output = self.kp * pitch_error - self.kd * pitch_rate

            # Apply torque limits
            control_output = max(min(control_output, self.max_torque), -self.max_torque)

            # Create joint command message
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [control_output]  # Simplified - real system would have multiple joints

            # Publish control command
            self.joint_cmd_pub.publish(cmd_msg)

            # Log control performance (only occasionally to avoid spam)
            if hasattr(self, '_log_counter'):
                self._log_counter += 1
            else:
                self._log_counter = 0

            if self._log_counter % 100 == 0:  # Log every 100 iterations
                self.get_logger().info(
                    f'Balance: pitch={self.current_pitch:.3f}, '
                    f'control={control_output:.3f}, '
                    f'freq={1.0/self.dt:.1f}Hz' if self.dt > 0 else 'freq=N/A'
                )

        except AttributeError:
            # Handle case where IMU data hasn't arrived yet
            pass
        except Exception as e:
            self.get_logger().error(f'Balance control error: {e}')

def main(args=None):
    rclpy.init(args=args)
    balance_controller = BalanceController()

    try:
        rclpy.spin(balance_controller)
    except KeyboardInterrupt:
        balance_controller.get_logger().info('Balance controller shutting down...')
    finally:
        balance_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Launch Files for Node Management

Create a launch directory and launch file:

```bash
mkdir launch
```

Create `launch/my_robot_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='my_robot_node',
            name='my_robot_node',
            output='screen',
            parameters=[
                # Add parameters here if needed
            ]
        ),
        Node(
            package='my_robot_package',
            executable='balance_controller',
            name='balance_controller',
            output='screen',
            parameters=[
                {'control_frequency': 100},  # 100Hz for balance
                {'max_torque': 50.0},
                {'balance_kp': 15.0},
                {'balance_kd': 2.0}
            ]
        )
    ])
```

## Building and Running Your Package

### Step 7: Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_package
source install/setup.bash
```

### Step 8: Run the Package

```bash
# Run a single node
ros2 run my_robot_package my_robot_node

# Or run with launch file
ros2 launch my_robot_package my_robot_launch.py
```

## Testing Your Package

Create `test/test_my_robot_node.py`:

```python
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from my_robot_package.my_robot_node import MyRobotNode

class TestMyRobotNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = MyRobotNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def test_node_creation(self):
        """Test that the node was created successfully"""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'my_robot_node')

if __name__ == '__main__':
    unittest.main()
```

## Constitution Alignment

This chapter addresses several constitutional requirements:

### Production-Ready Code (Key Standard III)
- Proper build system configurations (package.xml, setup.py)
- Error handling and logging
- Parameter configuration for different deployments

### Real-Time Validation (Principle IV)
- QoS profile configuration for low-latency control
- High-frequency control loops for bipedal balance feedback
- Timing considerations for real-time applications

### Target Hardware Optimization
- Code optimized for Jetson Orin Nano computational constraints
- Efficient memory usage and processing patterns

## Best Practices for Humanoid Robot Packages

### 1. Real-Time Safety
- Use appropriate QoS profiles for safety-critical communications
- Implement proper error handling and recovery
- Design control loops with known timing characteristics

### 2. Modularity
- Separate concerns into different nodes
- Use parameters for configuration
- Implement proper interfaces between components

### 3. Resource Management
- Monitor CPU and memory usage
- Implement efficient algorithms for embedded systems
- Use appropriate data structures for real-time performance

## Practical Examples

### Example 1: Vision Processing Node
A ROS 2 node for processing camera images:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')

        # Use appropriate QoS for camera data
        camera_qos = QoSProfile(
            depth=1,  # Only most recent image matters
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Can drop frames if needed
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            camera_qos
        )

        self.bridge = CvBridge()
        self.get_logger().info('Vision processor started')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Process image (example: edge detection)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Log processing time for performance monitoring
            self.get_logger().debug('Image processed successfully')

        except Exception as e:
            self.get_logger().error(f'Image processing error: {e}')

def main(args=None):
    rclpy.init(args=args)
    vision_processor = VisionProcessor()

    try:
        rclpy.spin(vision_processor)
    except KeyboardInterrupt:
        pass
    finally:
        vision_processor.destroy_node()
        rclpy.shutdown()
```

## Exercises

### Exercise 1: Package Creation
Create a ROS 2 package for a humanoid robot's walking controller. Include proper package.xml, setup.py, and a node that implements a simple walking gait with QoS profiles appropriate for locomotion control.

### Exercise 2: Parameter Configuration
Modify the balance controller node to accept additional parameters for different walking speeds and turning rates. Implement parameter validation to ensure safe operation.

### Exercise 3: Multi-Node System
Design a ROS 2 system with multiple nodes (balance control, vision processing, and high-level planning) that work together for humanoid navigation. Specify the communication patterns and QoS profiles between nodes.

## Summary

Creating ROS 2 packages in Python requires attention to proper build system configurations, QoS profiles for real-time applications, and production-ready code practices. For humanoid robots with strict timing constraints, these considerations are critical for safety and stability. Understanding the package structure and configuration files enables the development of modular, reusable robotics software that can be deployed on target hardware like the NVIDIA Jetson Orin Nano.

## Further Reading

- "ROS 2 Documentation" - Official ROS 2 tutorials and API documentation
- "Programming Robots with ROS" by Quigley et al.
- "Effective Robotics Programming with ROS" by Mahtani et al.
- "Real-Time Systems and Robotics" for timing-critical applications