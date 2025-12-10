# Data Model: ROS 2 Lessons

## Core Entities

### Lesson
- **ID**: Unique identifier for the lesson
- **Title**: Descriptive title of the lesson
- **Module**: Parent module this lesson belongs to
- **Chapter**: Chapter within the module
- **Learning Objectives**: List of objectives students should achieve
- **Content**: Markdown content of the lesson
- **Code Examples**: Associated code snippets and examples
- **Media**: Diagrams, screenshots, or other media assets
- **Citations**: APA-formatted citations used in the lesson
- **Prerequisites**: Prerequisites needed to understand the lesson
- **Success Criteria**: Specific outcomes to validate lesson comprehension

### Module
- **ID**: Unique identifier for the module
- **Title**: Descriptive title of the module
- **Description**: Overview of the module
- **Learning Path**: Order and relationship between chapters
- **Target Audience**: Intended audience (intermediate CS/AI background)
- **Duration**: Estimated time to complete the module
- **Lessons**: List of lessons contained in the module
- **Prerequisites**: Prerequisites needed to start the module

### Code Example
- **ID**: Unique identifier for the code example
- **Title**: Descriptive title of the example
- **Description**: Brief description of what the code demonstrates
- **Language**: Programming language (e.g., Python)
- **Dependencies**: ROS 2 packages or libraries required
- **Source Code**: Actual code content
- **Explanation**: Step-by-step explanation of the code
- **Use Case**: Scenario where this code would be applied
- **Output**: Expected output when running the code

### Citation
- **ID**: Unique identifier for the citation
- **APA Format**: Full citation in APA format
- **Source Type**: Academic paper, documentation, website, etc.
- **URL**: Link to the source if available
- **Verification Status**: Whether the source has been verified
- **Relevance**: How the source relates to the lesson content
- **Access Date**: Date the source was accessed

### Media Asset
- **ID**: Unique identifier for the media asset
- **Type**: Diagram, screenshot, video, etc.
- **Title**: Descriptive title
- **Description**: What the media asset shows/demonstrates
- **File Path**: Relative path from the documentation root
- **Alt Text**: Alternative text for accessibility
- **Related Lessons**: List of lessons that use this asset

## Relationships

### Module contains Lessons
- One module contains multiple lessons
- Each lesson belongs to one module
- Lessons have ordinal position within module (1, 2, 3, etc.)

### Lesson uses Code Examples
- One lesson can reference multiple code examples
- Each code example can be used by multiple lessons
- Examples are versioned to maintain consistency across lessons

### Lesson includes Citations
- One lesson can include multiple citations
- Each citation can be used in multiple lessons
- Citations are validated for APA format compliance

### Lesson utilizes Media Assets
- One lesson can use multiple media assets
- Each media asset can be used by multiple lessons
- Assets are optimized for web delivery

## Validation Rules

### Lesson Validation
- Title must be between 10-100 characters
- Content must include at least one code example
- Learning objectives must align with success criteria
- Must include 2-5 APA-formatted citations
- Content must pass plagiarism check

### Module Validation
- Must contain 2-4 lessons
- Duration must be between 2-4 hours
- Target audience must match specified demographic
- Learning path must be clearly defined

### Code Example Validation
- Must be executable in ROS 2 Humble environment
- Must include explanations for key concepts
- Output must be documented
- Dependencies must be specified

### Citation Validation
- Must follow APA format exactly
- Source must be accessible at time of publication
- Must be from an authoritative source
- Must be properly attributed