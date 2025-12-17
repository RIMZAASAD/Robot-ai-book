// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  textbookSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics Textbook',
      collapsible: true,
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Module 1: Foundations of Physical AI',
          collapsible: true,
          collapsed: false,
          items: [
            {
              type: 'doc',
              id: 'chapters/module-1-foundations/chapter-1-introduction-to-physical-ai',
              label: 'Chapter 1: Introduction to Physical AI'
            },
            {
              type: 'doc',
              id: 'chapters/module-1-foundations/chapter-2-embodied-intelligence',
              label: 'Chapter 2: Embodied Intelligence & Real-World Constraints'
            },
            {
              type: 'doc',
              id: 'chapters/module-1-foundations/chapter-3-humanoid-robotics-overview',
              label: 'Chapter 3: Humanoid Robotics Overview'
            },
            {
              type: 'doc',
              id: 'chapters/module-1-foundations/chapter-4-sensors-perception-systems',
              label: 'Chapter 4: Sensors & Perception Systems'
            }
          ]
        },
        {
          type: 'category',
          label: 'Module 2: ROS 2 - The Robotic Nervous System',
          collapsible: true,
          collapsed: false,
          items: [
            {
              type: 'doc',
              id: 'chapters/module-2-ros/chapter-5-ros2-architecture',
              label: 'Chapter 5: ROS 2 Architecture & Core Concepts'
            },
            {
              type: 'doc',
              id: 'chapters/module-2-ros/chapter-6-ros2-packages',
              label: 'Chapter 6: Creating ROS 2 Packages (Python)'
            },
            {
              type: 'doc',
              id: 'chapters/module-2-ros/chapter-7-urdf-xacro',
              label: 'Chapter 7: URDF & XACRO for Humanoid Robots'
            },
            {
              type: 'doc',
              id: 'chapters/module-2-ros/chapter-8-ros2-tools',
              label: 'Chapter 8: ROS 2 Tools: Rviz, RQt, TF2'
            }
          ]
        },
        {
          type: 'category',
          label: 'Module 3: Digital Twin Simulation',
          collapsible: true,
          collapsed: false,
          items: [
            {
              type: 'doc',
              id: 'chapters/module-3-simulation/chapter-9-gazebo-setup',
              label: 'Chapter 9: Gazebo Simulation Setup'
            },
            {
              type: 'doc',
              id: 'chapters/module-3-simulation/chapter-10-physics-sensor-simulation',
              label: 'Chapter 10: Physics & Sensor Simulation in Gazebo'
            },
            {
              type: 'doc',
              id: 'chapters/module-3-simulation/chapter-11-unity-hri',
              label: 'Chapter 11: Unity for Human-Robot Interaction'
            },
            {
              type: 'doc',
              id: 'chapters/module-3-simulation/chapter-12-isaac-sim-fundamentals',
              label: 'Chapter 12: NVIDIA Isaac Sim Fundamentals'
            },
            {
              type: 'doc',
              id: 'chapters/module-3-simulation/chapter-13-isaac-sdk',
              label: 'Chapter 13: Isaac SDK for Perception & Synthetic Data'
            }
          ]
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action Pipelines',
          collapsible: true,
          collapsed: false,
          items: [
            {
              type: 'doc',
              id: 'chapters/module-4-vla/chapter-14-computer-vision',
              label: 'Chapter 14: Computer Vision for Robotics'
            },
            {
              type: 'doc',
              id: 'chapters/module-4-vla/chapter-15-language-understanding',
              label: 'Chapter 15: Language Understanding in Robotics'
            },
            {
              type: 'doc',
              id: 'chapters/module-4-vla/chapter-16-action-planning',
              label: 'Chapter 16: Action Planning & Control Systems'
            },
            {
              type: 'doc',
              id: 'chapters/module-4-vla/chapter-17-vla-integration',
              label: 'Chapter 17: Integration: Vision-Language-Action Systems'
            },
            {
              type: 'doc',
              id: 'chapters/module-4-vla/chapter-18-capstone-project',
              label: 'Chapter 18: Capstone: Autonomous Humanoid Robot Project'
            }
          ]
        }
      ]
    }
  ]
};

export default sidebars;