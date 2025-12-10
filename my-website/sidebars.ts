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
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Tutorial',
      items: [
        'intro',
        'tutorial-basics/educational-components',
        'tutorial-extras/manage-docs-versions',
        'tutorial-extras/translate-your-site',
      ],
    },
  ],

  // Manual sidebar for ROS 2 Lessons modules
  ros2LessonsSidebar: [
    {
      type: 'category',
      label: 'ROS 2 Foundations',
      items: [
        'modules/ros2-foundations/index',
        'modules/ros2-foundations/lesson-1-architecture',
        'modules/ros2-foundations/lesson-2-nodes-topics-services'
      ],
      link: {
        type: 'doc',
        id: 'modules/ros2-foundations/index',
      },
    },
    {
      type: 'category',
      label: 'Python Integration',
      items: [
        'modules/python-integration/index',
        'modules/python-integration/lesson-1-writing-ros2-nodes-python',
        'modules/python-integration/lesson-2-bridging-ai-robot-controllers'
      ],
      link: {
        type: 'doc',
        id: 'modules/python-integration/index',
      },
    },
    {
      type: 'category',
      label: 'URDF Modeling',
      items: [
        'modules/urdf-modeling/index',
        'modules/urdf-modeling/lesson-1-urdf-structure-joint-definitions',
        'modules/urdf-modeling/lesson-2-building-basic-humanoid-model'
      ],
      link: {
        type: 'doc',
        id: 'modules/urdf-modeling/index',
      },
    },
  ],

  // Manual sidebar for Digital Twin (Gazebo & Unity) modules
  digitalTwinSidebar: [
    {
      type: 'category',
      label: 'Getting Started',
      items: [
        'getting-started',
        'integration',
        'workflows/gazebo-unity-integration',
        'resources',
        'glossary',
        'accessibility-checklist'
      ],
    },
    {
      type: 'category',
      label: 'Module 1: Simulation Basics',
      items: [
        'module-1-simulation-basics/intro',
        'module-1-simulation-basics/chapter-1-physics-concepts',
        'module-1-simulation-basics/chapter-2-gravity-and-collisions'
      ],
      link: {
        type: 'doc',
        id: 'module-1-simulation-basics/intro',
      },
    },
    {
      type: 'category',
      label: 'Module 2: Gazebo Environment',
      items: [
        'module-2-gazebo-environment/intro',
        'module-2-gazebo-environment/chapter-1-world-construction'
      ],
      link: {
        type: 'doc',
        id: 'module-2-gazebo-environment/intro',
      },
    },
    {
      type: 'category',
      label: 'Module 3: Unity Interaction',
      items: [
        'module-3-unity-interaction/intro',
        'module-3-unity-interaction/chapter-1-rendering'
      ],
      link: {
        type: 'doc',
        id: 'module-3-unity-interaction/intro',
      },
    },
    {
      type: 'category',
      label: 'Module 4: Sensor Simulation',
      items: [
        'module-4-sensor-simulation/intro',
        'module-4-sensor-simulation/chapter-1-lidar',
        'module-4-sensor-simulation/chapter-2-depth-camera',
        'module-4-sensor-simulation/chapter-3-imu-noise'
      ],
      link: {
        type: 'doc',
        id: 'module-4-sensor-simulation/intro',
      },
    },
    {
      type: 'category',
      label: 'Isaac Sim Modules',
      items: [
        'isaac-sim/intro',
        'isaac-sim/architecture-components',
        'isaac-sim/basic-scene-creation',
        'isaac-sim/physics-configuration',
        'isaac-sim/sensor-simulation',
        'isaac-sim/synthetic-data-generation'
      ],
      link: {
        type: 'doc',
        id: 'isaac-sim/intro',
      },
    },
    {
      type: 'category',
      label: 'Vision-Language-Action (VLA) Systems',
      items: [
        'vla-systems/intro',
        'vla-systems/chapter-1-foundations',
        'vla-systems/chapter-2-voice-action',
        'vla-systems/chapter-3-cognitive-planning',
        'vla-systems/capstone-project',
        'vla-systems/resources'
      ],
      link: {
        type: 'doc',
        id: 'vla-systems/intro',
      },
    }
  ],
};

export default sidebars;
