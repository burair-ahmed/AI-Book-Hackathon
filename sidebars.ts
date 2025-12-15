import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: Nervous System',
      items: [
        'module-01-nervous-system/foundation-physical-ai',
        'module-01-nervous-system/ros2-nodes',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twins',
      items: [
        'module-02-digital-twins/urdf-gazebo',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Isaac Brain',
      items: [
        'module-03-isaac-brain/isaac-ros',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA',
      items: [
        'module-04-vla/vla-control',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Capstone',
      items: [
        'module-05-capstone/capstone',
      ],
    },
  ],
};

export default sidebars;
