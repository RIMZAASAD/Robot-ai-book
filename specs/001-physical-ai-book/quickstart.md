# Quickstart: Physical AI Book Development

**Feature**: Physical AI Book Development
**Date**: 2025-12-06
**Branch**: 001-physical-ai-book

## Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager
- Git for version control
- A code editor (VS Code recommended)

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Navigate to the Docusaurus Directory
```bash
cd website  # Navigate to the Docusaurus project directory
```

### 3. Install Dependencies
```bash
npm install
# OR if using yarn
yarn install
```

### 4. Start Development Server
```bash
npm run start
# OR if using yarn
yarn start
```

This will start a local development server at `http://localhost:3000` with hot-reloading enabled.

## Project Structure

```
website/                    # Docusaurus project root
├── blog/                  # Optional blog content
├── docs/                  # Documentation content
│   ├── chapters/          # Chapter directories
│   │   ├── chapter1/      # Chapter 1 lessons
│   │   │   ├── lesson1.mdx
│   │   │   ├── lesson2.mdx
│   │   │   └── lesson3.mdx
│   │   └── chapter1.mdx   # Chapter 1 introduction
├── src/                   # Custom React components
├── static/                # Static assets (images, files)
├── docusaurus.config.js   # Main Docusaurus configuration
├── sidebars.js            # Navigation sidebar configuration
├── package.json           # Project dependencies and scripts
└── ...
```

## Creating New Content

### Adding a New Lesson
1. Create a new `.mdx` file in the appropriate chapter directory
2. Add frontmatter with title, description, and navigation settings
3. Write your lesson content using Markdown and JSX
4. Update `sidebars.js` to include the new lesson in navigation

Example lesson file (`docs/chapters/chapter1/new-lesson.mdx`):
```md
---
title: 'New Lesson Title'
description: 'Brief description of the lesson content'
sidebar_label: 'New Lesson'
sidebar_position: 4
---

# New Lesson Title

Content goes here...
```

### Adding a New Chapter
1. Create a new directory under `docs/chapters/`
2. Create a main chapter file (e.g., `chapter2.mdx`)
3. Create lesson files within the chapter directory
4. Update `sidebars.js` to include the new chapter

## Configuration Files

### docusaurus.config.js
Main configuration file for the site:
- Site metadata (title, tagline, URL)
- Theme configuration
- Plugin settings
- Deployment settings

### sidebars.js
Navigation structure:
- Defines the sidebar organization
- Controls the order of chapters and lessons
- Sets up navigation hierarchy

## Development Commands

```bash
# Start development server
npm run start

# Build static site for production
npm run build

# Serve built site locally for testing
npm run serve

# Deploy to configured hosting (e.g., GitHub Pages)
npm run deploy
```

## Content Guidelines

### Writing Lessons
- Use clear, descriptive headings (H1 for title, H2/H3 for sections)
- Format code blocks with proper language specification
- Include relevant images with alt text
- Link to related content within the book
- Follow consistent formatting and style

### Frontmatter Requirements
Each content file should include:
- `title`: Display title for the page
- `description`: SEO description
- `sidebar_label`: Navigation label (if different from title)
- `sidebar_position`: Order in navigation

## Building and Deployment

### Local Build
```bash
npm run build
```
This creates a `build/` directory with the static site ready for deployment.

### Deployment
The built site can be deployed to:
- GitHub Pages
- Netlify
- Vercel
- Any static hosting service

For GitHub Pages deployment, ensure `docusaurus.config.js` is configured with the correct `organizationName`, `projectName`, and `deploymentBranch`.

## Troubleshooting

### Common Issues
- **Port already in use**: Change port with `npm run start -- --port 3001`
- **Build errors**: Check for syntax errors in MDX files and configuration
- **Navigation not updating**: Restart development server after sidebar changes

### Development Tips
- Changes to content files are reflected immediately
- Changes to configuration files may require server restart
- Use the development server's error messages to identify issues quickly