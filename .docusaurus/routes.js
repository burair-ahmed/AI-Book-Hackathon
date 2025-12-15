import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/physical-ai-textbook/__docusaurus/debug',
    component: ComponentCreator('/physical-ai-textbook/__docusaurus/debug', 'b91'),
    exact: true
  },
  {
    path: '/physical-ai-textbook/__docusaurus/debug/config',
    component: ComponentCreator('/physical-ai-textbook/__docusaurus/debug/config', '992'),
    exact: true
  },
  {
    path: '/physical-ai-textbook/__docusaurus/debug/content',
    component: ComponentCreator('/physical-ai-textbook/__docusaurus/debug/content', 'f3c'),
    exact: true
  },
  {
    path: '/physical-ai-textbook/__docusaurus/debug/globalData',
    component: ComponentCreator('/physical-ai-textbook/__docusaurus/debug/globalData', 'e4e'),
    exact: true
  },
  {
    path: '/physical-ai-textbook/__docusaurus/debug/metadata',
    component: ComponentCreator('/physical-ai-textbook/__docusaurus/debug/metadata', '5b8'),
    exact: true
  },
  {
    path: '/physical-ai-textbook/__docusaurus/debug/registry',
    component: ComponentCreator('/physical-ai-textbook/__docusaurus/debug/registry', 'e4d'),
    exact: true
  },
  {
    path: '/physical-ai-textbook/__docusaurus/debug/routes',
    component: ComponentCreator('/physical-ai-textbook/__docusaurus/debug/routes', '500'),
    exact: true
  },
  {
    path: '/physical-ai-textbook/docs',
    component: ComponentCreator('/physical-ai-textbook/docs', '33f'),
    routes: [
      {
        path: '/physical-ai-textbook/docs',
        component: ComponentCreator('/physical-ai-textbook/docs', '150'),
        routes: [
          {
            path: '/physical-ai-textbook/docs',
            component: ComponentCreator('/physical-ai-textbook/docs', '860'),
            routes: [
              {
                path: '/physical-ai-textbook/docs/intro',
                component: ComponentCreator('/physical-ai-textbook/docs/intro', '83d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module-01-nervous-system/foundation-physical-ai',
                component: ComponentCreator('/physical-ai-textbook/docs/module-01-nervous-system/foundation-physical-ai', 'f82'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module-01-nervous-system/ros2-nodes',
                component: ComponentCreator('/physical-ai-textbook/docs/module-01-nervous-system/ros2-nodes', 'd08'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module-02-digital-twins/urdf-gazebo',
                component: ComponentCreator('/physical-ai-textbook/docs/module-02-digital-twins/urdf-gazebo', 'e59'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module-03-isaac-brain/isaac-ros',
                component: ComponentCreator('/physical-ai-textbook/docs/module-03-isaac-brain/isaac-ros', 'f60'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module-04-vla/vla-control',
                component: ComponentCreator('/physical-ai-textbook/docs/module-04-vla/vla-control', '2c2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-textbook/docs/module-05-capstone/capstone',
                component: ComponentCreator('/physical-ai-textbook/docs/module-05-capstone/capstone', '12e'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
