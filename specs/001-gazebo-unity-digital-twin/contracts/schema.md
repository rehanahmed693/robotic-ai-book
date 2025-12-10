# Docusaurus Configuration Schema for Digital Twin Educational Content

## Overview
This document defines the configuration schema for the Docusaurus-based educational website focused on Digital Twin simulation using Gazebo and Unity. The schema defines how content is organized, how navigation works, and how educational features are structured.

## Site Configuration Schema (docusaurus.config.ts)

### Required Properties
- `title`: string - The title of the educational site
- `tagline`: string - A brief tagline describing the content
- `favicon`: string - Path to the site favicon

### Theme Configuration
- `navbar`: object - Navigation bar configuration
  - `title`: string - Title displayed in the navbar
  - `logo`: object - Logo configuration 
    - `alt`: string - Alt text for the logo
    - `src`: string - Path to the logo image
  - `items`: array - Navigation items
    - `type`: "doc" | "docsVersionDropdown" | "localeDropdown" | "search" | "custom" - Type of navigation item
    - `docId`: string - ID of the document (for type: "doc")
    - `position`: "left" | "right" - Position in the navbar
    - `label`: string - Display label for the navigation item
- `footer`: object - Footer configuration
  - `style`: "dark" | "light" - Footer style
  - `links`: array - Footer links
    - `title`: string - Title for the link group
    - `items`: array - Individual links
      - `label`: string - Link text
      - `to`: string - Link destination
  - `copyright`: string - Copyright text

### Plugins Configuration
- `@docusaurus/plugin-content-docs`: object - Documentation plugin configuration
  - `id`: string - Plugin instance ID
  - `path`: string - Path to documentation files
  - `routeBasePath`: string - Base URL for docs
  - `sidebarPath`: string - Path to sidebar configuration
  - `editUrl`: string - Base URL for editing content
  - `showLastUpdateTime`: boolean - Whether to show last update time
  - `docItemComponent`: string - Component for doc pages

## Content Structure Schema

### Module Level
```
docs/
├── module-1-simulation-basics/
│   ├── chapter-1-physics-concepts.md
│   ├── chapter-2-gravity-and-collisions.md
│   └── _category_.json
├── module-2-gazebo-environment/
│   ├── chapter-1-world-setup.md
│   └── _category_.json
├── module-3-unity-interaction/
│   ├── chapter-1-rendering.md
│   └── _category_.json
└── module-4-sensor-simulation/
    ├── chapter-1-lidar.md
    └── _category_.json
```

### File Metadata Schema
Each Markdown file should include frontmatter with:
- `title`: string - The page title
- `sidebar_label`: string - Label to display in sidebar (optional, defaults to title)
- `description`: string - Page description for SEO
- `keywords`: array of strings - Keywords for search
- `learning_objectives`: array of strings - What students will learn
- `duration`: number - Estimated time to complete in minutes

### Category Configuration (_category_.json)
- `label`: string - Display label for the category
- `collapsible`: boolean - Whether the category is collapsible in sidebar
- `collapsed`: boolean - Whether the category is collapsed by default
- `link`: object (optional) - Link configuration for the category
  - `type`: "doc" | "generated-index" - Type of link
  - `id`: string - Document ID (for type: "doc")
  - `title`: string - Page title (for type: "generated-index")
  - `description`: string - Page description (for type: "generated-index")
  - `slug`: string - Custom slug (for type: "generated-index")

## Navigation Schema (sidebars.ts)

### Sidebar Structure
```
module.exports = {
  someSidebar: {
    [module]: [
      {
        type: 'category',
        label: [chapter],
        items: [lesson IDs],
        collapsed: false,  // Chapters are expanded by default
      },
    ],
  },
};
```

### Category Properties
- `type`: "category" - Identifies this as a category
- `label`: string - Display name for the category
- `items`: array - Array of doc IDs or nested categories
- `collapsed`: boolean - Whether the category is collapsed by default

## Educational Features Schema

### Learning Objective Component
A React component to display learning objectives:
- `objectives`: array of strings - List of learning objectives
- `style`: "list" | "card" - Display style

### Duration Estimator Component
A component to display estimated reading time:
- `minutes`: number - Estimated time in minutes
- `activity`: "reading" | "exercise" | "project" - Type of activity

### Assessment Component Schema
A component for quizzes and exercises:
- `type`: "multiple-choice" | "true-false" | "short-answer" | "practical" - Type of assessment
- `question`: string - The question text
- `options`: array of strings (for multiple choice) - Answer options
- `correctAnswer`: string | number | boolean - The correct answer
- `explanation`: string - Explanation of the correct answer

## Technical Validation Requirements

### Content Format
- All content must be in Markdown format (`.md` extension)
- Images must be in web-appropriate formats (`.png`, `.jpg`, `.gif`, `.svg`)
- Code examples must be properly formatted with language identifiers

### Accessibility Requirements
- All images must have appropriate alt text
- Headings must follow proper hierarchy (h1, h2, h3, etc.)
- Links must be descriptive and contextually appropriate
- Color contrast must meet WCAG 2.1 AA standards

### SEO Requirements
- Each page must have a unique title and description
- Proper use of heading hierarchy for content structure
- Appropriate use of keywords without stuffing

## Deployment Schema
- Build process must generate static HTML, CSS, and JavaScript
- Site must be deployable to static hosting (GitHub Pages, Netlify, Vercel, etc.)
- Must include proper 404 page
- Must include sitemap.xml and robots.txt