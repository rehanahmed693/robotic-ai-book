# Data Model: Docusaurus Book Architecture for Digital Twin (Gazebo & Unity)

## Overview
This data model describes the information architecture for the educational content focused on Digital Twin simulation using Gazebo and Unity. The structure follows a hierarchical model of Modules → Chapters → Lessons to guide student learning.

## Entity: Module
- **Name**: String (required) - The module title
- **Description**: String (required) - Brief summary of the module's learning objectives
- **Chapters**: Array of Chapter entities (required, 1-5 items)
- **LearningObjectives**: Array of String (required) - What students will learn in this module
- **Prerequisites**: Array of String (optional) - What knowledge students should have before starting
- **Duration**: Integer (optional) - Estimated time to complete in minutes

### Relationships
- Module contains 1 to many Chapters
- Module belongs to the overall course structure

## Entity: Chapter
- **Name**: String (required) - The chapter title
- **Description**: String (required) - Brief summary of the chapter's content
- **Lessons**: Array of Lesson entities (required, 1-4 items)
- **LearningObjectives**: Array of String (required) - What students will learn in this chapter
- **Duration**: Integer (optional) - Estimated time to complete in minutes

### Relationships
- Chapter belongs to 1 Module
- Chapter contains 1 to many Lessons

## Entity: Lesson
- **Title**: String (required) - The lesson title
- **Content**: String (required) - The main content of the lesson in Markdown format
- **LearningObjectives**: Array of String (required) - What students will learn in this lesson
- **Activities**: Array of String (optional) - Hands-on exercises for students
- **Assessment**: Array of String (optional) - Questions to verify understanding
- **Duration**: Integer (optional) - Estimated time to complete in minutes
- **Resources**: Array of Resource references (optional) - Additional materials
- **Media**: Array of Media entities (optional) - Diagrams, screenshots, etc.
- **CodeExamples**: Array of CodeExample entities (optional) - For technical implementation

### Relationships
- Lesson belongs to 1 Chapter
- Lesson can reference multiple Resources
- Lesson can contain multiple Media elements
- Lesson can contain multiple CodeExamples

## Entity: Resource
- **Title**: String (required) - The resource title
- **URL**: String (required) - The link to the resource
- **Description**: String (optional) - Brief explanation of the resource
- **Type**: String (required) - The type of resource (e.g., "documentation", "video", "code", "paper")

## Entity: Media
- **Type**: String (required) - The type of media ("image", "diagram", "screenshot", "video")
- **AltText**: String (required) - Alternative text for accessibility
- **Caption**: String (optional) - Caption text for the media
- **Path**: String (required) - File path to the media asset
- **Description**: String (optional) - Detailed description of the media content

## Entity: CodeExample
- **Title**: String (optional) - Title for the code example
- **Language**: String (required) - Programming language
- **Code**: String (required) - The actual code content
- **Description**: String (optional) - Explanation of what the code does
- **FileName**: String (optional) - Suggested file name if applicable

## Validation Rules
- Module name must be between 5 and 100 characters
- Chapter name must be between 5 and 100 characters
- Lesson title must be between 5 and 100 characters
- Each Module must contain at least 1 Chapter and no more than 5 Chapters
- Each Chapter must contain at least 1 Lesson and no more than 4 Lessons
- All content must follow APA citation style for references
- LearningObjectives must be specific, measurable, and achievable
- Duration values must be positive integers or null

## State Transitions
The content model follows a linear progression from Module → Chapter → Lesson, with each level building on the previous. There are no state transitions in the traditional sense, but students progress through the content in the defined order.

## Constraints
- All content must be written in Markdown format for Docusaurus compatibility
- All technical claims must be source-traceable according to project constitution
- Content must maintain Flesch-Kincaid grade level between 11-13
- All media assets must be appropriately licensed for educational use
- All code examples must be tested and verified for accuracy