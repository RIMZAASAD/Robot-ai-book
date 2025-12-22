---
id: 5
title: Docusaurus-project-structure-update
stage: plan
date: 2025-12-06
surface: agent
model: Claude Sonnet 4.5
feature: 001-physical-ai-book
branch: 001-physical-ai-book
user: user
command: /sp.plan
labels: [docusaurus, structure, project, update]
links:
  spec: ./specs/001-physical-ai-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - ./specs/001-physical-ai-book/plan.md
 - ./specs/001-physical-ai-book/research_updated.md
tests:
 - null
---

## Prompt

update the folder structure docsouras project structure

## Response snapshot

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-book/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Docusaurus Documentation Structure
website/                   # Docusaurus project root
├── blog/                 # Optional blog content
├── docs/                 # Documentation source files
│   ├── chapters/         # Main chapters directory
│   │   ├── chapter1/     # Chapter 1 lessons directory
│   │   │   ├── lesson1.mdx    # Lesson 1 content
│   │   │   ├── lesson2.mdx    # Lesson 2 content
│   │   │   ├── lesson3.mdx    # Lesson 3 content
│   │   │   └── _category_.json # Chapter 1 category config
│   │   ├── chapter1.mdx       # Chapter 1 introduction page
│   │   └── _category_.json    # Chapters category config
│   ├── intro.mdx              # Introduction page
│   └── ...
├── src/                 # Custom React components and pages
│   ├── components/      # Reusable React components
│   │   ├── Admonition/  # Custom admonition components
│   │   └── Mermaid/     # Mermaid diagram components
│   ├── css/             # Custom styles
│   │   └── custom.css
│   └── pages/           # Custom pages
│       └── index.js     # Custom home page
├── static/              # Static assets (images, files)
│   └── img/             # Images and diagrams
│       ├── chapter1/    # Chapter 1 images
│       └── ...
├── docusaurus.config.js # Main Docusaurus configuration
├── sidebars.js          # Navigation sidebar configuration
├── package.json         # Project dependencies and scripts
├── babel.config.js      # Babel configuration
├── tsconfig.json        # TypeScript configuration (if using TS)
└── yarn.lock            # Yarn lock file (or package-lock.json if using npm)
```

**Structure Decision**: The Docusaurus documentation structure was chosen because it provides a proven framework for organizing content with built-in navigation, search, and responsive design. The chapter/lesson hierarchy follows Docusaurus best practices for nested documentation. The `_category_.json` files provide proper categorization and navigation grouping for chapters and lessons.

# Research: Docusaurus Project Structure for Physical AI Book

**Feature**: Physical AI Book Development
**Date**: 2025-12-06
**Branch**: 001-physical-ai-book

## Decision: Docusaurus Project Structure

**Rationale**: The updated Docusaurus project structure follows official Docusaurus best practices for organizing documentation with nested categories and proper navigation. The `_category_.json` files provide automatic navigation grouping and consistent styling for chapters and lessons.

## Decision: Content Organization with Categories

**Rationale**: Using `_category_.json` files in the chapters directory provides:
- Automatic sidebar navigation grouping
- Consistent styling for chapter/lesson hierarchy
- Proper navigation breadcrumbs
- Configurable category settings (collapsible, position, etc.)

## Decision: Custom Components for Constitution Compliance

**Rationale**: Creating custom components for Admonitions and Mermaid diagrams ensures compliance with the Physical AI Book constitution requirements:
- Admonitions (note, tip, warning) for highlighting critical insights
- Mermaid diagrams for visualizing complex spatial and architectural concepts

## Decision: Static Assets Organization

**Rationale**: Organizing static assets in `static/img/chapter1/` structure provides:
- Clear separation of images by chapter
- Maintainable asset management
- Scalable structure for future chapters
- Consistent with Docusaurus static asset handling

## Technology Stack Research

### Docusaurus Version and Configuration
- **Selected**: Docusaurus 2.x with modern preset
- **Configuration files**: `docusaurus.config.js`, `sidebars.js`, `babel.config.js`
- **Package management**: npm or yarn with lock files for reproducible builds

### Navigation Structure
- **Sidebar organization**: Hierarchical navigation using `sidebars.js`
- **Category navigation**: Automatic grouping with `_category_.json` files
- **Breadcrumb navigation**: Generated automatically based on structure

## Implementation Best Practices

### Content Structure
1. **Directory structure**: Organized by chapters with nested lessons
2. **File naming**: Consistent naming conventions (kebab-case)
3. **Frontmatter**: Standardized metadata for all MDX files
4. **Navigation**: Clear hierarchy with proper positioning

### Performance Considerations
1. **Image optimization**: Proper sizing and formats in static/img/
2. **Code splitting**: Docusaurus handles automatically
3. **Bundle size**: Monitor with built-in tools

### Accessibility
1. **Semantic structure**: Proper heading hierarchy (H1, H2, H3)
2. **Navigation**: Keyboard and screen reader accessible
3. **Images**: Proper alt text and descriptions

## Outcome

- ✅ Impact: Updated Docusaurus project structure with detailed folder organization including _category_.json files for proper navigation
- 🧪 Tests: No specific tests required for this structural update
- 📁 Files: Updated plan.md and created research_updated.md with comprehensive structure details
- 🔁 Next prompts: Implementation of the defined structure
- 🧠 Reflection: Structure now follows Docusaurus best practices and includes provisions for constitution compliance

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementing the defined structure