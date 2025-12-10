---
id: "001"
title: "Generate Tasks for ROS2 Lessons Feature"
stage: "tasks"
date_iso: "2025-12-09"
surface: "agent"
model: "Qwen"
feature: "001-ros2-lessons"
branch: "001-ros2-lessons"
user: "hasse"
command: "$ARGUMENTS"
labels: ["tasks", "specification", "ros2"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "specs/001-ros2-lessons/tasks.md"
tests_yaml: []
prompt_text: |
  description: Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts.

  You **MUST** consider the user input before proceeding (if not empty).

  Outline

  1. Setup: Run `.specify/scripts/powershell/check-prerequisites.ps1 -Json` from repo root and parse FEATURE_DIR and AVAILABLE_DOCS list. All paths must be absolute. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

  2. Load design documents: Read from FEATURE_DIR:
     - Required: plan.md (tech stack, libraries, structure), spec.md (user stories with priorities)
     - Optional: data-model.md (entities), contracts/ (API endpoints), research.md (decisions), quickstart.md (test scenarios)
     - Note: Not all projects have all documents. Generate tasks based on what's available.

  3. Execute task generation workflow:
     - Load plan.md and extract tech stack, libraries, project structure
     - Load spec.md and extract user stories with their priorities (P1, P2, P3, etc.)
     - If data-model.md exists: Extract entities and map to user stories
     - If contracts/ exists: Map endpoints to user stories
     - If research.md exists: Extract decisions for setup tasks
     - Generate tasks organized by user story (see Task Generation Rules below)
     - Generate dependency graph showing user story completion order
     - Create parallel execution examples per user story
     - Validate task completeness (each user story has all needed tasks, independently testable)

  4. Generate tasks.md: Use .specify.specify/templates/tasks-template.md as structure, fill with:
     - Correct feature name from plan.md
     - Phase 1: Setup tasks (project initialization)
     - Phase 2: Foundational tasks (blocking prerequisites for all user stories)
     - Phase 3+: One phase per user story (in priority order from spec.md)
     - Each phase includes: story goal, independent test criteria, tests (if requested), implementation tasks
     - Final Phase: Polish & cross-cutting concerns
     - All tasks must follow the strict checklist format (see Task Generation Rules below)
     - Clear file paths for each task
     - Dependencies section showing story completion order
     - Parallel execution examples per story
     - Implementation strategy section (MVP first, incremental delivery)

  5. Report: Output path to generated tasks.md and summary:
     - Total task count
     - Task count per user story
     - Parallel opportunities identified
     - Independent test criteria for each story
     - Suggested MVP scope (typically just User Story 1)
     - Format validation: Confirm ALL tasks follow the checklist format (checkbox, ID, labels, file paths)

  Context for task generation: 

  The tasks.md should be immediately executable - each task must be specific enough that an LLM can complete it without additional context.

  Task Generation Rules

  CRITICAL: Tasks MUST be organized by user story to enable independent implementation and testing.

  Tests are OPTIONAL: Only generate test tasks if explicitly requested in the feature specification or if user requests TDD approach.

  Checklist Format (REQUIRED)

  Every task MUST strictly follow this format:

  text
  - [ ] [TaskID] [P?] [Story?] Description with file path


  Format Components:

  1. Checkbox: ALWAYS start with - [ ] (markdown checkbox)
  2. Task ID: Sequential number (T001, T002, T003...) in execution order
  3. [P] marker: Include ONLY if task is parallelizable (different files, no dependencies on incomplete tasks)
  4. [Story] label: REQUIRED for user story phase tasks only
     - Format: [US1], [US2], [US3], etc. (maps to user stories from spec.md)
     - Setup phase: NO story label
     - Foundational phase: NO story label  
     - User Story phases: MUST have story label
     - Polish phase: NO story label
  5. Description: Clear action with exact file path

  Examples:

  - ✅ CORRECT: - [ ] T001 Create project structure per implementation plan
  - ✅ CORRECT: - [ ] T005 [P] Implement authentication middleware in src/middleware/auth.py
  - ✅ CORRECT: - [ ] T012 [P] [US1] Create User model in src/models/user.py
  - ✅ CORRECT: - [ ] T014 [US1] Implement UserService in src/services/user_service.py
  - ❌ WRONG: - [ ] Create User model (missing ID and Story label)
  - ❌ WRONG: T001 [US1] Create model (missing checkbox)
  - ❌ WRONG: - [ ] [US1] Create User model (missing Task ID)
  - ❌ WRONG: - [ ] T001 [US1] Create model (missing file path)

  Task Organization

  1. From User Stories (spec.md) - PRIMARY ORGANIZATION:
     - Each user story (P1, P2, P3...) gets its own phase
     - Map all related components to their story:
       - Models needed for that story
       - Services needed for that story
       - Endpoints/UI needed for that story
       - If tests requested: Tests specific to that story
     - Mark story dependencies (most stories should be independent)

  2. From Contracts:
     - Map each contract/endpoint → to the user story it serves
     - If tests requested: Each contract → contract test task [P] before implementation in that story's phase

  3. From Data Model:
     - Map each entity to the user story(ies) that need it
     - If entity serves multiple stories: Put in earliest story or Setup phase
     - Relationships → service layer tasks in appropriate story phase

  4. From Setup/Infrastructure:
     - Shared infrastructure → Setup phase (Phase 1)
     - Foundational/blocking tasks → Foundational phase (Phase 2)
     - Story-specific setup → within that story's phase

  Phase Structure

  - Phase 1: Setup (project initialization)
  - Phase 2: Foundational (blocking prerequisites - MUST complete before user stories)
  - Phase 3+: User Stories in priority order (P1, P2, P3...)
    - Within each story: Tests (if requested) → Models → Services → Endpoints → Integration
    - Each phase should be a complete, independently testable increment
  - Final Phase: Polish & Cross-Cutting Concerns

  As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

  1) Determine Stage
     - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

  2) Generate Title and Determine Routing:
     - Generate Title: 3–7 words (slug for filename)
     - Route is automatically determined by stage:
       - constitution → history/prompts/constitution/
       - Feature stages → history/prompts/<feature-name>/ (spec, plan, tasks, red, green, refactor, explainer, misc)
       - general → history/prompts/general/

  3) Create and Fill PHR (Shell first; fallback agent‑native)
     - Run: .specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json
     - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
     - If the script fails:
       - Read .specify/templates/phr-template.prompt.md (or templates/…)
       - Allocate an ID; compute the output path based on stage from step 2; write the file
       - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

  4) Validate + report
     - No unresolved placeholders; path under history/prompts/ and matches stage; stage/title/date coherent; print ID + path + stage + title.
     - On failure: warn, don't block. Skip only for /sp.phr.
response_text: "Generated tasks.md file for the ROS 2 Lessons feature in C:\\Users\\hasse\\Desktop\\robotic-project\\specs\\001-ros2-lessons\\tasks.md. The file contains 53 tasks organized by phase and research topic priority (P1, P2, P3), following the required checklist format with proper IDs, story labels, and file paths. Tasks are organized into phases: 1) Research Setup, 2) Foundational Setup, 3) Research Topic 1 (ROS 2 Foundations), 4) Research Topic 2 (Python Integration), 5) Research Topic 3 (URDF Modeling), 6) API Integration, and N) Polish & QA. Each research topic can be developed independently with its own lessons, code examples, and validation criteria. Dependencies and parallel execution opportunities are clearly outlined."
outcome: "Successfully generated a comprehensive tasks.md file that follows all the specified requirements and organizational structure for the ROS2 lessons feature."
evaluation: "The task was completed successfully with all requirements met: proper format validation, correct organization by research topics, inclusion of all necessary details, and adherence to the template structure."
---

# Tasks: ROS 2 Lessons - Robotic Nervous System

**Input**: Design documents from `/specs/001-ros2-lessons/`
**Prerequisites**: plan.md (required), spec.md (required for research topics), research.md

**Validation**: The examples below include validation tasks. Validation is OPTIONAL - only include them if explicitly requested in the research specification.

**Organization**: Tasks are grouped by research topic to enable independent completion and validation of each topic.

## Format: `[ID] [P?] [Topic] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Topic]**: Which research topic this task belongs to (e.g., RT1, RT2, RT3)
- Include exact file paths in descriptions

## Path Conventions

- **Documentation**: `my-website/docs/modules/` for the Docusaurus-based lessons
- **Configuration**: `my-website/` for docusaurus.config.ts, sidebars.ts
- **Code examples**: `my-website/docs/modules/*/code-examples/` for ROS 2 code examples
- **Media assets**: `my-website/static/img/` for diagrams and screenshots

<!--
  ============================================================================
  IMPORTANT: This task list was generated based on:
  - Research topics from spec.md (with their priorities P1, P2, P3...)
  - Research requirements from plan.md
  - Key concepts from spec.md
  - Literature review from research.md
  - Data model from data-model.md
  - Quickstart guide from quickstart.md
  ============================================================================

  Tasks are organized by research topic so each topic can be:
  - Researched independently
  - Validated independently
  - Delivered as a research contribution increment
  ============================================================================
-->

## Phase 1: Research Setup (Foundation)

**Purpose**: ROS 2 lessons project initialization and foundational setup

- [ ] T001 Create ROS 2 lessons project structure per implementation plan in specs/001-ros2-lessons/
- [ ] T002 [P] Initialize Docusaurus documentation site in my-website/ with modules directory
- [ ] T003 [P] Configure citation management with APA format compliance in bibliography/

---

## Phase 2: Foundational Setup (Blocking Prerequisites)

**Purpose**: Core setup that MUST be complete before ANY research topic can be addressed

**⚠️ CRITICAL**: No research topic work can begin until this phase is complete

- [ ] T004 [P] Set up Docusaurus with modules navigation structure in my-website/sidebars.ts
- [ ] T005 [P] Configure Docusaurus for ROS 2 lessons theme in my-website/docusaurus.config.ts
- [ ] T006 [P] Create module index files for the three main modules in my-website/docs/modules/
- [ ] T007 Create basic directory structure for code examples in my-website/docs/modules/*/code-examples/
- [ ] T008 [P] Create media assets directory structure in my-website/static/img/
- [ ] T009 Set up development environment following quickstart.md guidelines

**Checkpoint**: Documentation foundation ready - research topic work can now begin in parallel

---

## Phase 3: Research Topic 1 - ROS 2 Foundations (Priority: P1) 🎯 Core Contribution

**Goal**: Create educational content covering the foundational concepts of ROS 2 as middleware for humanoid robot control, focusing on architecture and core communication patterns (nodes, topics, services).

**Independent Validation**: Students will demonstrate understanding by creating a simple publisher-subscriber pair and calling a service they've defined, showing they can establish basic ROS 2 communication patterns.

### Research for Topic 1

- [ ] T010 [P] [RT1] Create lesson-1-architecture.md in my-website/docs/modules/ros2-foundations/
- [ ] T011 [P] [RT1] Create lesson-2-nodes-topics-services.md in my-website/docs/modules/ros2-foundations/
- [ ] T012 [P] [RT1] Develop publisher-subscriber code example in my-website/docs/modules/ros2-foundations/code-examples/
- [ ] T013 [P] [RT1] Develop service client/server code example in my-website/docs/modules/ros2-foundations/code-examples/
- [ ] T014 [RT1] Create ROS 2 architecture diagram in my-website/static/img/ and reference in lesson-1-architecture.md
- [ ] T015 [RT1] Add learning objectives and success criteria to both lessons in my-website/docs/modules/ros2-foundations/
- [ ] T016 [RT1] Integrate 2-5 APA-formatted citations in both lessons
- [ ] T017 [RT1] Add prerequisites and code explanations to lesson content
- [ ] T018 [RT1] Perform technical accuracy review for ROS 2 Foundations content

**Checkpoint**: At this point, Research Topic 1 should be complete and independently validatable

---

## Phase 4: Research Topic 2 - Python-to-ROS Control (rclpy) (Priority: P2)

**Goal**: Create educational content covering the use of the rclpy Python client library to create ROS 2 nodes and bridge AI agents with robot controllers.

**Independent Validation**: Students will demonstrate by creating a Python node that communicates with other ROS nodes and implements a basic control algorithm connecting AI decision-making to robot actions.

### Research for Topic 2

- [ ] T019 [P] [RT2] Create lesson-1-writing-ros2-nodes-python.md in my-website/docs/modules/python-integration/
- [ ] T020 [P] [RT2] Create lesson-2-bridging-ai-robot-controllers.md in my-website/docs/modules/python-integration/
- [ ] T021 [P] [RT2] Develop Python node implementation code example in my-website/docs/modules/python-integration/code-examples/
- [ ] T022 [P] [RT2] Develop AI-to-robot control bridge code example in my-website/docs/modules/python-integration/code-examples/
- [ ] T023 [RT2] Create rclpy integration diagram in my-website/static/img/ and reference in lesson-1-writing-ros2-nodes-python.md
- [ ] T024 [RT2] Add learning objectives and success criteria to both lessons in my-website/docs/modules/python-integration/
- [ ] T025 [RT2] Integrate 2-5 APA-formatted citations in both lessons
- [ ] T026 [RT2] Add prerequisites and code explanations to lesson content
- [ ] T027 [RT2] Perform technical accuracy review for Python Integration content

**Checkpoint**: At this point, Research Topics 1 AND 2 should both be independently valid

---

## Phase 5: Research Topic 3 - Humanoid Robot Modeling (URDF) (Priority: P3)

**Goal**: Create educational content covering understanding and creating URDF (Unified Robot Description Format) files to model humanoid robots, including joint definitions and kinematic chains.

**Independent Validation**: Students will demonstrate by creating a URDF file representing a basic humanoid model and validating it can be parsed by ROS tools.

### Research for Topic 3

- [ ] T028 [P] [RT3] Create lesson-1-urdf-structure-joint-definitions.md in my-website/docs/modules/urdf-modeling/
- [ ] T029 [P] [RT3] Create lesson-2-building-basic-humanoid-model.md in my-website/docs/modules/urdf-modeling/
- [ ] T030 [P] [RT3] Develop basic URDF structure code example in my-website/docs/modules/urdf-modeling/code-examples/
- [ ] T031 [P] [RT3] Develop complete humanoid model URDF example in my-website/docs/modules/urdf-modeling/code-examples/
- [ ] T032 [RT3] Create URDF modeling diagram in my-website/static/img/ and reference in lesson-1-urdf-structure-joint-definitions.md
- [ ] T033 [RT3] Add learning objectives and success criteria to both lessons in my-website/docs/modules/urdf-modeling/
- [ ] T034 [RT3] Integrate 2-5 APA-formatted citations in both lessons
- [ ] T035 [RT3] Add prerequisites and code explanations to lesson content
- [ ] T036 [RT3] Perform technical accuracy review for URDF Modeling content

**Checkpoint**: All research topics should now be independently valid

---

## Phase 6: API Integration & Student Tracking

**Goal**: Implement the API endpoints defined in the contracts directory to support student progress tracking and lesson assessments.

- [ ] T037 [P] Implement GET /modules/{module_id}/lessons endpoint
- [ ] T038 [P] Implement GET /lessons/{lesson_id} endpoint
- [ ] T039 [P] Implement POST /students/{student_id}/progress endpoint
- [ ] T040 [P] Implement GET /students/{student_id}/progress endpoint
- [ ] T041 [P] Implement POST /lessons/{lesson_id}/assessments endpoint
- [ ] T042 Add authentication middleware for API endpoints
- [ ] T043 Create database schemas for lessons, students, and progress tracking
- [ ] T044 Integrate API with the Docusaurus frontend to enable progress tracking

---

## Phase N: Polish & Quality Assurance

**Purpose**: Improvements that synthesize all research topics and ensure quality standards

- [ ] T045 [P] Create comprehensive quickstart guide in my-website/docs/quickstart.md
- [ ] T046 [P] Create FAQ section for each module in my-website/docs/modules/*/faq.md
- [ ] T047 [P] Add cross-references between related lessons in different modules
- [ ] T048 Create troubleshooting guide for common ROS 2 environment issues
- [ ] T049 Validate all lessons meet 3-4 session duration target from spec.md
- [ ] T050 Conduct user testing with target audience (intermediate CS/AI background)
- [ ] T051 [P] Verify all citations follow APA format in bibliography/
- [ ] T052 Conduct accessibility review of all content and media
- [ ] T053 Final review of all content for technical accuracy and clarity

---

## Dependencies & Execution Order

### Phase Dependencies

- **Research Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational Setup (Phase 2)**: Depends on Setup completion - BLOCKS all research topics
- **Research Topics (Phase 3+)**: All depend on Foundational phase completion
  - Research topics can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **API Integration (Phase 6)**: Can proceed in parallel with Research Topics (Phase 3-5)
- **Polish & QA (Final Phase)**: Depends on all desired research topics being complete

### Research Topic Dependencies

- **Research Topic 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other topics
- **Research Topic 2 (P2)**: Can start after Foundational (Phase 2) - May build on basic ROS 2 concepts from RT1 but should be independently researchable
- **Research Topic 3 (P3)**: Can start after Foundational (Phase 2) - May reference communication patterns from RT1 but should be independently researchable

### Within Each Research Topic

- Create lesson structure first
- Add content with learning objectives and success criteria
- Develop code examples for each lesson
- Create supporting media assets
- Add citations and references
- Review for technical accuracy

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all research topics can start in parallel (if team capacity allows)
- All lessons within a research topic marked [P] can run in parallel
- All code examples within a research topic marked [P] can run in parallel
- Media creation for different topics can run in parallel
- Different research topics can be worked on in parallel by different team members

---

## Parallel Example: Research Topic 1

```bash
# Launch all lesson creation tasks for Research Topic 1 together:
Task: "Create lesson-1-architecture.md in my-website/docs/modules/ros2-foundations/"
Task: "Create lesson-2-nodes-topics-services.md in my-website/docs/modules/ros2-foundations/"

# Launch all code examples for Research Topic 1 together:
Task: "Develop publisher-subscriber code example in my-website/docs/modules/ros2-foundations/code-examples/"
Task: "Develop service client/server code example in my-website/docs/modules/ros2-foundations/code-examples/"
```

---

## Research Strategy

### Core First (Research Topic 1 Only)

1. Complete Phase 1: Research Setup
2. Complete Phase 2: Foundational Setup (CRITICAL - blocks all topics)
3. Complete Phase 3: Research Topic 1
4. **STOP and VALIDATE**: Verify Research Topic 1 independently
5. Document preliminary findings if ready

### Incremental Delivery

1. Complete Setup + Foundational → Documentation foundation ready
2. Add Research Topic 1 → Validate independently → Document (Core contribution!)
3. Add Research Topic 2 → Validate independently → Document
4. Add Research Topic 3 → Validate independently → Document
5. Each topic adds value without invalidating previous topics

### Parallel Team Strategy

With multiple researchers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Researcher A: Research Topic 1 (ROS 2 Foundations)
   - Researcher B: Research Topic 2 (Python Integration)
   - Researcher C: Research Topic 3 (URDF Modeling)
   - Researcher D: API Integration
3. Topics complete and validate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Topic] label maps task to specific research topic for traceability
- Each research topic should be independently completable and validatable
- Verify validation criteria are defined before research begins
- Commit after each task or logical group
- Stop at any checkpoint to validate topic independently
- Avoid: vague tasks, same file conflicts, cross-topic dependencies that break independence
- Ensure all work complies with the ROS 2 Lessons Specification