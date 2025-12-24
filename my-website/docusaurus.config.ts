import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Book',
  tagline: 'Comprehensive Educational Program in Advanced Robotics & AI',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://robotic-project.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'robotic-project', // Usually your GitHub org/user name.
  projectName: 'robotic-project', // Usually your repo name.

  onBrokenLinks: 'warn',
  markdown: {
    format: 'mdx',
    mermaid: false,
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/robotic-project/robotic-project/tree/main/',
          beforeDefaultRemarkPlugins: [
            // Additional remark plugins
          ],
          beforeDefaultRehypePlugins: [
            // Additional rehype plugins
          ],
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Robotics',
      logo: {
        alt: 'Digital Twin Lessons Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'digitalTwinSidebar',
          position: 'left',
          label: 'Modules',
        },
        {
          type: 'dropdown',
          label: 'Resources',
          position: 'left',
          items: [
            {
              type: 'doc',
              label: 'Isaac Sim Guide',
              docId: 'isaac-sim/intro',
            },
            {
              type: 'doc',
              label: 'ROS 2 Lessons',
              docId: 'modules/ros2-foundations/index',
            },
            {
              type: 'doc',
              label: 'Gazebo & Unity Integration',
              docId: 'integration',
            }
          ],
        },
        {
          type: 'search',
          position: 'right',
        },
        {
          href: 'https://github.com/robotic-project/robotic-project',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Modules',
          items: [
            {
              label: 'Simulation Basics',
              to: '/docs/module-1-simulation-basics/intro',
            },
            {
              label: 'Gazebo Environment',
              to: '/docs/module-2-gazebo-environment/intro',
            },
            {
              label: 'Unity Interaction',
              to: '/docs/module-3-unity-interaction/intro',
            },
            {
              label: 'Sensor Simulation',
              to: '/docs/module-4-sensor-simulation/intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'ROS Answers',
              href: 'https://answers.ros.org/',
            },
            {
              label: 'Gazebo Community',
              href: 'https://community.gazebosim.org/',
            },
            {
              label: 'Unity Learn',
              href: 'https://learn.unity.com/',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/robotic-project/robotic-project',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Course. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'json', 'markdown'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;