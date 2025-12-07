import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar configuration for the module-based curriculum
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      items: [
        'module-01-ros2/architecture',
        'module-01-ros2/nodes-and-topics',
        'module-01-ros2/urdf-basics'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin',
      items: [
        'module-02-digital-twin/gazebo-setup',
        'module-02-digital-twin/unity-integration'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain',
      items: [
        'module-03-robot-brain/isaac-sim',
        'module-03-robot-brain/nav2-slam'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-04-vla/voice-control',
        'module-04-vla/llm-reasoning'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 5: Capstone Project',
      items: [
        'module-05-capstone/final-project'
      ],
      collapsed: false,
    },
  ],
};

export default sidebars;
