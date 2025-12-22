// @ts-check
import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'From Simulation to Embodied Intelligence',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://robotic-ai-book-kappa.vercel.app', // Vercel deployment URL
  // Set the /<baseUrl>/ pathname under which your site is served
  // For Vercel, this is typically the root path
  baseUrl: '/',

  organizationName: 'RIMZAASAD', // GitHub org/user name (not used by Vercel)
  projectName: 'Robotic-ai-Book', // Repo name (not used by Vercel)

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',
  trailingSlash: false,

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      ({
        docs: {
          sidebarPath: './sidebars.js',
          editUrl:
            'https://github.com/RIMZAASAD/Robotic-ai-Book/edit/main/website/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics Textbook',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Textbook',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar',
          position: 'left',
          label: 'Physical AI Book',
        },
        {
          href: 'https://github.com/RIMZAASAD/Robotic-ai-Book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Physical AI Book',
          items: [
            {
              label: 'Physical AI Book',
              to: '/docs/chapters/module-1-foundations/chapter-1-introduction-to-physical-ai',
            },
          ],
        },
        {
          title: 'Social Media',
          items: [
            {
              label: 'Instagram',
              href: 'https://www.instagram.com/rimza218/',
            },
            {
              label: 'LinkedIn',
              href: 'https://www.linkedin.com/in/rimza-asad-206b332b8/',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/RIMZAASAD',
            },
          ],
        },
        {
          title: 'Additional',
          items: [
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/users/30621843/rimza-asad',
            },
          ],
        },
      ],
      copyright: `Copyright © 2025 Physical AI & Humanoid Robotics Textbook, Built by Rimza`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },
};

export default config;


