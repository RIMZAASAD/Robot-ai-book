# Data Model: Physical AI & Humanoid Robotics — Textbook

**Feature**: 001-physical-ai-textbook
**Date**: 2025-12-10

## Content Entities

### Chapter
- **Fields**:
  - id: string (e.g., "module-1-chapter-1")
  - title: string (e.g., "Introduction to Physical AI")
  - module: reference to Module
  - number: integer (chapter number within module)
  - content: string (Markdown content)
  - learningObjectives: array of strings
  - keyConcepts: array of strings
  - examples: array of Example objects
  - exercises: array of Exercise objects
  - prerequisites: array of string references to other chapters
  - assets: array of Asset references

- **Validation**:
  - title must be 5-100 characters
  - content must be in Markdown format
  - learningObjectives must have 2-5 items
  - number must be positive

### Module
- **Fields**:
  - id: string (e.g., "module-1-foundations")
  - title: string (e.g., "Foundations of Physical AI")
  - number: integer (module number 1-4)
  - description: string
  - chapters: array of Chapter references
  - prerequisites: array of string references to other modules
  - capstoneProject: boolean

- **Validation**:
  - title must be 5-100 characters
  - number must be 1-4
  - chapters must have 1-6 items (for this textbook)
  - description must be 20-500 characters

### Example
- **Fields**:
  - id: string
  - title: string
  - description: string
  - code: string (optional)
  - expectedOutcome: string
  - difficulty: enum (beginner, intermediate, advanced)

- **Validation**:
  - title must be 5-100 characters
  - difficulty must be one of specified values

### Exercise
- **Fields**:
  - id: string
  - title: string
  - description: string
  - instructions: string
  - expectedOutcome: string
  - difficulty: enum (beginner, intermediate, advanced)
  - verificationSteps: array of strings

- **Validation**:
  - title must be 5-100 characters
  - difficulty must be one of specified values
  - verificationSteps must have 1-10 items

### Asset
- **Fields**:
  - id: string
  - filename: string
  - type: enum (image, diagram, video, code, document)
  - description: string
  - path: string (relative to assets directory)
  - altText: string (for accessibility)

- **Validation**:
  - filename must have valid extension for type
  - path must be relative and not contain '..'
  - altText required for image/diagram types

### CapstoneProject
- **Fields**:
  - id: string
  - title: string
  - description: string
  - modulesIntegrated: array of Module references
  - requirements: array of strings
  - deliverables: array of strings
  - evaluationCriteria: array of strings

- **Validation**:
  - title must be 5-100 characters
  - modulesIntegrated must reference 2-4 modules
  - evaluationCriteria must have 3-10 items

## Relationships

- Module 1--* Chapter (one module contains many chapters)
- Chapter 1--* Example (one chapter contains many examples)
- Chapter 1--* Exercise (one chapter contains many exercises)
- Chapter 1--* Asset (one chapter references many assets)
- CapstoneProject 1--* Module (capstone integrates multiple modules)

## State Transitions

### Chapter States
- draft → review → approved → published
- published → updated → review (when content is modified)

### Module States
- planning → draft → review → approved → published
- published → updated → review (when chapters are modified)

## Validation Rules from Requirements

1. **Content Structure Requirements**:
   - All content must be in Markdown format
   - Each chapter must include learning objectives, key concepts, practical examples, and exercises
   - Content must be beginner-friendly while maintaining technical accuracy

2. **Accessibility Requirements**:
   - Content must be accessible without physical robotics hardware
   - Simulations and examples must work in virtual environments

3. **Success Criteria**:
   - Each chapter fully covers its intended course outline requirements
   - Content is suitable for RAG chatbot indexing
   - Content is structured in Docusaurus-ready Markdown format