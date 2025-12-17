# Research: Docusaurus Project Structure for Physical AI Book

**Feature**: Physical AI Book Development
**Date**: 2025-12-06
**Branch**: 001-physical-ai-book

## Decision: Docusaurus Project Structure

**Rationale**: The updated Docusaurus project structure follows official Docusaurus best practices for organizing documentation with nested categories and proper navigation. The `_category_.json` files provide automatic navigation grouping and consistent styling for chapters and lessons.

**Alternatives considered**:
1. **Flat structure**: All files in docs/ root - would not provide proper navigation hierarchy
2. **Custom structure**: Non-standard directory organization - would not follow Docusaurus conventions
3. **Simple nested**: Directories without `_category_.json` - would lack proper navigation grouping

## Decision: Content Organization with Categories

**Rationale**: Using `_category_.json` files in the chapters directory provides:
- Automatic sidebar navigation grouping
- Consistent styling for chapter/lesson hierarchy
- Proper navigation breadcrumbs
- Configurable category settings (collapsible, position, etc.)

**Example `_category_.json` structure**:
```json
{
  "label": "Chapter 1: Introduction to Physical AI",
  "position": 1,
  "link": {
    "type": "doc",
    "id": "chapters/chapter1"
  }
}
```

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