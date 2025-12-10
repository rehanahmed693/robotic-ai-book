---
title: Accessibility Verification Checklist
sidebar_label: Accessibility Checklist
description: Checklist for verifying accessibility compliance in educational content
keywords: [accessibility, compliance, education, checklist, wcag, content]
learning_objectives:
  - Apply accessibility standards to educational content
  - Verify compliance with WCAG 2.1 AA standards
  - Ensure content is accessible to all learners
duration: 10
---

import LearningObjectives from '@site/src/components/LearningObjectives';
import DurationEstimator from '@site/src/components/DurationEstimator';

<LearningObjectives objectives={[
  'Apply accessibility standards to educational content',
  'Verify compliance with WCAG 2.1 AA standards',
  'Ensure content is accessible to all learners'
]} />

<DurationEstimator minutes={10} activity="reading" />

# Accessibility Verification Checklist

This checklist ensures that all educational content meets accessibility standards, specifically WCAG 2.1 AA requirements.

## Content Structure

- [ ] All pages have a clear, descriptive title
- [ ] Headings follow proper hierarchy (h1, h2, h3, etc.)
- [ ] Content is organized in a logical sequence
- [ ] Sufficient contrast between text and background (4.5:1 minimum for normal text)
- [ ] Text can be resized up to 200% without loss of content or functionality

## Images and Media

- [ ] All images have appropriate alt text describing the content and function
- [ ] Decorative images have empty alt attributes (`alt=""`)
- [ ] Complex images (diagrams, charts) have detailed descriptions
- [ ] Captions are provided for videos
- [ ] Audio content has transcripts available

## Navigation and Interaction

- [ ] All content is accessible via keyboard navigation
- [ ] Focus indicators are visible when navigating with keyboard
- [ ] Links have descriptive text (not just "click here")
- [ ] Consistent navigation is provided across all pages
- [ ] Users can skip repetitive content (e.g., skip to main content links)

## Forms and Input

- [ ] All form fields have associated labels
- [ ] Error messages are clear and helpful
- [ ] Instructions are provided for complex forms

## Technical Implementation

- [ ] HTML is properly structured and valid
- [ ] Semantic markup is used appropriately (e.g., `<nav>`, `<main>`, `<section>`)
- [ ] ARIA labels and roles are used where necessary
- [ ] Color is not used as the only means of conveying information
- [ ] Timeouts or automatic redirects can be extended or turned off

## Testing

- [ ] Content has been tested with screen readers
- [ ] Keyboard navigation has been tested
- [ ] Color contrast has been validated
- [ ] Alternative text has been verified for accuracy
- [ ] Forms have been tested for usability with assistive technologies