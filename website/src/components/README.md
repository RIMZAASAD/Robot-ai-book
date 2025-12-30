# Enhanced UI Components for Physical AI & Humanoid Robotics Textbook

This directory contains enhanced UI components for the Docusaurus-based textbook website, featuring modern design elements with the existing purple/blue color scheme.

## Components

### 1. EnhancedCard
A modern card component with multiple variants and hover effects:
- **Variants**: default, gradient, glass, elevated
- **Features**: Hover effects, clickable cards, icon support
- **Usage**: For displaying content, features, and information cards

### 2. ContentCard
A specialized card for displaying educational content:
- **Features**: Level indicators (beginner/intermediate/advanced), duration, tags
- **Usage**: For displaying chapters, modules, and learning materials
- **Interactive**: Hover effects and navigation support

### 3. ModernHero
A modern hero section component:
- **Features**: Animated text, multiple button support, scroll indicator
- **Usage**: For homepage and landing page headers
- **Responsive**: Fully responsive design

### 4. ModernSidebar
An enhanced sidebar navigation component:
- **Features**: Collapsible categories, active state highlighting
- **Usage**: For documentation navigation
- **Interactive**: Expandable/collapsible categories

### 5. ModernFooter
A modern footer component:
- **Features**: Multi-column layout, social links, responsive design
- **Usage**: For the website footer
- **Accessible**: Proper focus states and keyboard navigation

### 6. ModernSearch
An enhanced search component:
- **Features**: Live suggestions, keyboard navigation, dropdown results
- **Usage**: For improved search functionality
- **Interactive**: Hover and active states

## Integration

The components are integrated into the existing Docusaurus structure:
- Homepage uses ModernHero and enhanced cards
- Documentation pages use the custom sidebar layout
- All components respect the existing dark theme and purple/blue color scheme
- Responsive design maintained across all screen sizes

## Styling

All components follow the existing design system:
- Dark theme with purple/blue accents
- Glassmorphism effects
- Smooth animations and transitions
- Consistent spacing and typography
- Accessibility features included

## Usage Examples

### ContentCard
```jsx
<ContentCard
  title="Introduction to Physical AI"
  description="Learn the fundamental concepts of Physical AI and how it differs from traditional AI approaches."
  to="/docs/chapters/module-1-foundations/chapter-1-introduction-to-physical-ai"
  icon="🧠"
  level="beginner"
  duration="30 min"
  tags={['AI', 'Fundamentals', 'Physical Intelligence']}
/>
```

### EnhancedCard
```jsx
<EnhancedCard
  title="🧠 Embodied Intelligence"
  description="Dive deep into how AI interacts with the physical world, moving beyond digital brains to physical existence."
  variant="glass"
  hoverEffect={true}
/>
```

## Responsive Design

All components are fully responsive and adapt to different screen sizes:
- Desktop: Full layout with sidebar and multiple columns
- Tablet: Adjusted spacing and layout
- Mobile: Single column layout with hamburger menu for sidebar

## Accessibility

- Proper focus states for keyboard navigation
- Semantic HTML elements
- ARIA attributes where appropriate
- Reduced motion support
- Color contrast compliant with WCAG standards