# Data Model: Physical AI Book

**Feature**: Physical AI Book Development
**Date**: 2025-12-06
**Branch**: 001-physical-ai-book

## Entities

### Chapter
- **Description**: A primary organizational unit of the book, containing multiple lessons
- **Attributes**:
  - `id`: String - Unique identifier for the chapter (e.g., "chapter1")
  - `title`: String - Display title of the chapter
  - `description`: String - Brief description of the chapter content
  - `lessons`: Array of Lesson entities - The lessons contained within this chapter
  - `metadata`: Object - Additional information for Docusaurus configuration

### Lesson
- **Description**: A sub-unit within a chapter, focusing on a specific topic
- **Attributes**:
  - `id`: String - Unique identifier for the lesson (e.g., "lesson1")
  - `title`: String - Display title of the lesson
  - `description`: String - Brief description of the lesson content
  - `content`: String - The main content of the lesson in MDX format
  - `chapterId`: String - Reference to the parent chapter
  - `order`: Number - Position of the lesson within the chapter
  - `metadata`: Object - Additional information for Docusaurus configuration (frontmatter)

### Book
- **Description**: The overarching collection of chapters and lessons, organized for Docusaurus
- **Attributes**:
  - `title`: String - The overall title of the book
  - `description`: String - Brief description of the entire book
  - `chapters`: Array of Chapter entities - All chapters in the book
  - `navigation`: Object - Configuration for Docusaurus sidebar navigation

## Relationships

### Chapter -> Lesson (One-to-Many)
- A Chapter contains multiple Lessons
- Each Lesson belongs to exactly one Chapter
- The relationship is defined by the `chapterId` attribute in Lesson and the `lessons` array in Chapter

### Book -> Chapter (One-to-Many)
- A Book contains multiple Chapters
- Each Chapter belongs to exactly one Book
- The relationship is defined by the `chapters` array in Book

## Validation Rules

### Chapter Validation
- `title` must be non-empty string (max 100 characters)
- `description` must be non-empty string (max 500 characters)
- `lessons` array must contain 1-10 Lesson entities
- `id` must be unique within the Book scope

### Lesson Validation
- `title` must be non-empty string (max 100 characters)
- `description` must be non-empty string (max 300 characters)
- `content` must be non-empty string (min 100 characters for meaningful content)
- `order` must be a positive integer
- `chapterId` must reference an existing Chapter

### Book Validation
- `title` must be non-empty string (max 100 characters)
- `description` must be non-empty string (max 500 characters)
- `chapters` array must contain 1-20 Chapter entities initially (extensible)

## State Transitions

### Chapter States
- `draft`: Chapter is being created/edited
- `review`: Chapter is under review
- `published`: Chapter is ready for public consumption

### Lesson States
- `draft`: Lesson is being created/edited
- `review`: Lesson is under review
- `published`: Lesson is ready for public consumption

## Docusaurus-Specific Considerations

### Frontmatter Structure
Each Chapter and Lesson MDX file will include frontmatter with:
- `title`: The display title
- `description`: SEO and navigation description
- `sidebar_label`: Label to appear in sidebar navigation
- `sidebar_position`: Order in sidebar
- `keywords`: SEO keywords
- `image`: Social sharing image

### Navigation Structure
The sidebar navigation will be organized hierarchically:
```
- Chapter 1
  - Lesson 1
  - Lesson 2
  - Lesson 3
- Chapter 2 (future)
  - Lesson 1 (future)
  - ...
```