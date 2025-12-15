import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/AI-Book-Hackathon/__docusaurus/debug',
    component: ComponentCreator('/AI-Book-Hackathon/__docusaurus/debug', '838'),
    exact: true
  },
  {
    path: '/AI-Book-Hackathon/__docusaurus/debug/config',
    component: ComponentCreator('/AI-Book-Hackathon/__docusaurus/debug/config', 'd1a'),
    exact: true
  },
  {
    path: '/AI-Book-Hackathon/__docusaurus/debug/content',
    component: ComponentCreator('/AI-Book-Hackathon/__docusaurus/debug/content', '291'),
    exact: true
  },
  {
    path: '/AI-Book-Hackathon/__docusaurus/debug/globalData',
    component: ComponentCreator('/AI-Book-Hackathon/__docusaurus/debug/globalData', '692'),
    exact: true
  },
  {
    path: '/AI-Book-Hackathon/__docusaurus/debug/metadata',
    component: ComponentCreator('/AI-Book-Hackathon/__docusaurus/debug/metadata', 'ef4'),
    exact: true
  },
  {
    path: '/AI-Book-Hackathon/__docusaurus/debug/registry',
    component: ComponentCreator('/AI-Book-Hackathon/__docusaurus/debug/registry', '99b'),
    exact: true
  },
  {
    path: '/AI-Book-Hackathon/__docusaurus/debug/routes',
    component: ComponentCreator('/AI-Book-Hackathon/__docusaurus/debug/routes', '88b'),
    exact: true
  },
  {
    path: '/AI-Book-Hackathon/docs',
    component: ComponentCreator('/AI-Book-Hackathon/docs', '34d'),
    routes: [
      {
        path: '/AI-Book-Hackathon/docs',
        component: ComponentCreator('/AI-Book-Hackathon/docs', '67f'),
        routes: [
          {
            path: '/AI-Book-Hackathon/docs',
            component: ComponentCreator('/AI-Book-Hackathon/docs', '286'),
            routes: [
              {
                path: '/AI-Book-Hackathon/docs/intro',
                component: ComponentCreator('/AI-Book-Hackathon/docs/intro', '6dc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI-Book-Hackathon/docs/module-01-nervous-system/foundation-physical-ai',
                component: ComponentCreator('/AI-Book-Hackathon/docs/module-01-nervous-system/foundation-physical-ai', '0cf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI-Book-Hackathon/docs/module-01-nervous-system/ros2-nodes',
                component: ComponentCreator('/AI-Book-Hackathon/docs/module-01-nervous-system/ros2-nodes', '096'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI-Book-Hackathon/docs/module-02-digital-twins/urdf-gazebo',
                component: ComponentCreator('/AI-Book-Hackathon/docs/module-02-digital-twins/urdf-gazebo', '07b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI-Book-Hackathon/docs/module-03-isaac-brain/isaac-ros',
                component: ComponentCreator('/AI-Book-Hackathon/docs/module-03-isaac-brain/isaac-ros', '8d2'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI-Book-Hackathon/docs/module-04-vla/vla-control',
                component: ComponentCreator('/AI-Book-Hackathon/docs/module-04-vla/vla-control', '192'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/AI-Book-Hackathon/docs/module-05-capstone/capstone',
                component: ComponentCreator('/AI-Book-Hackathon/docs/module-05-capstone/capstone', 'a0a'),
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
    path: '/AI-Book-Hackathon/',
    component: ComponentCreator('/AI-Book-Hackathon/', '413'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
