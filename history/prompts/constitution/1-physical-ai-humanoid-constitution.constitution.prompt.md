---
id: 1
title: Physical AI Humanoid Constitution
stage: constitution
date: 2025-12-09
surface: agent
model: Qwen
feature: none
branch: main
user: hasse
command: /sp.constitution
labels: [constitution, research-paper, ai, robotics]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
 - .specify/templates/spec-template.md
 - .specify/templates/tasks-template.md
tests:
 - none
---

## Prompt

Create or update the project constitution from interactive or provided principle inputs, ensuring all dependent templates stay in sync

## User Input

```text
/sp.constitution Project:
Research paper on Physical AI & Humanoid Robotics

Core principles:

Accuracy through robotics/AI documentation and primary source verification

Clarity for academic readers (CS/Robotics background)

Reproducibility of all technical claims, workflows, and system descriptions

Rigor (peer-reviewed robotics/AI/HRI sources preferred)

Key standards:

All claims must be source-traceable

Citation format: APA style

At least 50% peer-reviewed articles or conference papers

Plagiarism check: 0% tolerance

Writing clarity: Flesch-Kincaid grade 11–13

Constraints:

Word count: 6,000–8,500

Minimum 20 sources

Format: PDF with embedded citations

Required coverage:

ROS 2 (nodes, topics, URDF, rclpy)

Gazebo & Unity (physics, sensors, rendering)

NVIDIA Isaac (perception, SLAM, navigation)

Vision-Language-Action (Whisper, LLM planning)

Capstone pipeline: voice → plan → navigate → detect → manipulate

Success criteria:

All claims verified

Zero plagiarism

Passes technical fact-checking review
```

## Response snapshot

I have created and updated the project constitution for your Physical AI & Humanoid Robotics research paper. The changes include:

1. Created a new constitution document at `.specify/memory/constitution.md` with:
   - Core principles: Accuracy, Clarity, Reproducibility, and Rigor
   - Key standards: Source tracing, APA citation format, peer-reviewed sources requirement
   - Technical requirements: Coverage of ROS 2, Gazebo/Unity, NVIDIA Isaac, and Vision-Language-Action systems
   - Success criteria: Claim verification, plagiarism compliance, and technical fact-checking

2. Updated dependent templates to align with the research paper constitution:
   - Modified `.specify/templates/plan-template.md` to include constitution check for research paper requirements
   - Updated `.specify/templates/spec-template.md` to reflect research paper structure instead of feature implementation
   - Modified `.specify/templates/tasks-template.md` to support research paper tasks instead of feature implementation

All files have been updated to ensure consistency with the research paper focus and constitutional principles.

## Outcome

- ✅ Impact: Established comprehensive constitution for Physical AI & Humanoid Robotics research paper with aligned templates
- 🧪 Tests: none
- 📁 Files: Updated constitution.md and 3 templates (plan, spec, tasks)
- 🔁 Next prompts: Ready for research paper specification and planning
- 🧠 Reflection: The templates were successfully adapted from software development to academic research context

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): standard constitution creation
- Next experiment (smallest change to try): Begin research paper specification based on constitution
