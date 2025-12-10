---

description: "Task list template for research paper implementation"
---

# Tasks: [RESEARCH PAPER TITLE]

**Input**: Design documents from `/specs/[###-paper-title]/`
**Prerequisites**: plan.md (required), spec.md (required for research topics), research.md

**Validation**: The examples below include validation tasks. Validation is OPTIONAL - only include them if explicitly requested in the research specification.

**Organization**: Tasks are grouped by research topic to enable independent completion and validation of each topic.

## Format: `[ID] [P?] [Topic] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Topic]**: Which research topic this task belongs to (e.g., RT1, RT2, RT3)
- Include exact file paths in descriptions

## Path Conventions

- **Research paper**: `paper/`, `literature-review/`, `bibliography/` at repository root
- **Simulation studies**: `gazebo-studies/`, `unity-comparisons/`
- **Code examples**: `code-examples/`, `scripts/`
- Paths shown below assume single project - adjust based on plan.md structure

<!--
  ============================================================================
  IMPORTANT: The tasks below are SAMPLE TASKS for illustration purposes only.

  The /sp.tasks command MUST replace these with actual tasks based on:
  - Research topics from spec.md (with their priorities P1, P2, P3...)
  - Research requirements from plan.md
  - Key concepts from spec.md
  - Literature review from research.md

  Tasks MUST be organized by research topic so each topic can be:
  - Researched independently
  - Validated independently
  - Delivered as a research contribution increment

  DO NOT keep these sample tasks in the generated tasks.md file.
  ============================================================================
-->

## Phase 1: Research Setup (Foundation)

**Purpose**: Research project initialization and literature foundation

- [ ] T001 Create research project structure per implementation plan
- [ ] T002 Establish literature database with primary sources on Physical AI & Humanoid Robotics
- [ ] T003 [P] Configure citation management with APA format compliance

---

## Phase 2: Foundational Research (Blocking Prerequisites)

**Purpose**: Core research that MUST be complete before ANY research topic can be addressed

**⚠️ CRITICAL**: No research topic work can begin until this phase is complete

Examples of foundational tasks (adjust based on your project):

- [ ] T004 Establish baseline understanding of ROS 2 (nodes, topics, URDF, rclpy)
- [ ] T005 [P] Investigate Gazebo and Unity simulation frameworks (physics, sensors, rendering)
- [ ] T006 [P] Examine NVIDIA Isaac capabilities (perception, SLAM, navigation)
- [ ] T007 Research Vision-Language-Action systems (Whisper, LLM planning)
- [ ] T008 Define capstone pipeline methodology: voice → plan → navigate → detect → manipulate
- [ ] T009 Establish reproducibility standards for technical claims

**Checkpoint**: Research foundation ready - research topic work can now begin in parallel

---

## Phase 3: Research Topic 1 - [Title] (Priority: P1) 🎯 Core Contribution

**Goal**: [Brief description of what this research contributes]

**Independent Validation**: [How to verify this research is valid on its own]

### Validation for Research Topic 1 (OPTIONAL - only if validation requested) ⚠️

> **NOTE: Write these validation criteria FIRST, ensure they are defined before research begins**

- [ ] T010 [P] [RT1] Define reproducibility test for [technical claim] in paper/validation/[topic].md
- [ ] T011 [P] [RT1] Establish source-traceability requirements for [research area] in paper/validation/[topic].md

### Research for Topic 1

- [ ] T012 [P] [RT1] Conduct literature review on [Concept1] in literature-review/[concept1].md
- [ ] T013 [P] [RT1] Analyze [Concept2] implementations in literature-review/[concept2].md
- [ ] T014 [RT1] Synthesize findings on [research focus] in paper/sections/[focus-area].md (depends on T012, T013)
- [ ] T015 [RT1] Draft primary section on [topic] in paper/sections/[section].md
- [ ] T016 [RT1] Add APA citations and reference verification
- [ ] T017 [RT1] Perform technical accuracy review for research topic 1

**Checkpoint**: At this point, Research Topic 1 should be complete and independently validatable

---

## Phase 4: Research Topic 2 - [Title] (Priority: P2)

**Goal**: [Brief description of what this research contributes]

**Independent Validation**: [How to verify this research is valid on its own]

### Validation for Research Topic 2 (OPTIONAL - only if validation requested) ⚠️

- [ ] T018 [P] [RT2] Define reproducibility test for [technical claim] in paper/validation/[topic].md
- [ ] T019 [P] [RT2] Establish source-traceability requirements for [research area] in paper/validation/[topic].md

### Research for Topic 2

- [ ] T020 [P] [RT2] Conduct literature review on [Concept] in literature-review/[concept].md
- [ ] T021 [RT2] Analyze [research focus] in paper/sections/[analysis].md
- [ ] T022 [RT2] Draft section on [topic] in paper/sections/[section].md
- [ ] T023 [RT2] Integrate with Research Topic 1 findings (if needed)

**Checkpoint**: At this point, Research Topics 1 AND 2 should both be independently valid

---

## Phase 5: Research Topic 3 - [Title] (Priority: P3)

**Goal**: [Brief description of what this research contributes]

**Independent Validation**: [How to verify this research is valid on its own]

### Validation for Research Topic 3 (OPTIONAL - only if validation requested) ⚠️

- [ ] T024 [P] [RT3] Define reproducibility test for [technical claim] in paper/validation/[topic].md
- [ ] T025 [P] [RT3] Establish source-traceability requirements for [research area] in paper/validation/[topic].md

### Research for Topic 3

- [ ] T026 [P] [RT3] Conduct literature review on [Concept] in literature-review/[concept].md
- [ ] T027 [RT3] Analyze [research focus] in paper/sections/[analysis].md
- [ ] T028 [RT3] Draft section on [topic] in paper/sections/[section].md

**Checkpoint**: All research topics should now be independently valid

---

[Add more research topic phases as needed, following the same pattern]

---

## Phase N: Paper Assembly & Quality Assurance

**Purpose**: Improvements that synthesize all research topics and ensure quality standards

- [ ] TXXX [P] Synthesize introduction and conclusion sections in paper/
- [ ] TXXX Perform comprehensive peer review across all sections
- [ ] TXXX Validate all technical claims meet reproducibility requirements
- [ ] TXXX [P] Verify all citations follow APA format in bibliography/
- [ ] TXXX Conduct plagiarism check on complete paper
- [ ] TXXX Run clarity assessment (Flesch-Kincaid grade level check)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Research Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational Research (Phase 2)**: Depends on Setup completion - BLOCKS all research topics
- **Research Topics (Phase 3+)**: All depend on Foundational phase completion
  - Research topics can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Paper Assembly (Final Phase)**: Depends on all desired research topics being complete

### Research Topic Dependencies

- **Research Topic 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other topics
- **Research Topic 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with RT1 but should be independently researchable
- **Research Topic 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with RT1/RT2 but should be independently researchable

### Within Each Research Topic

- Validation criteria (if included) MUST be defined before research begins
- Literature reviews before analysis
- Analysis before synthesis
- Core research before integration
- Topic complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all research topics can start in parallel (if team capacity allows)
- All validation tasks for a research topic marked [P] can run in parallel
- Literature reviews within a topic marked [P] can run in parallel
- Different research topics can be worked on in parallel by different team members

---

## Parallel Example: Research Topic 1

```bash
# Launch all validation tasks for Research Topic 1 together (if validation requested):
Task: "Define reproducibility test for [technical claim] in paper/validation/[topic].md"
Task: "Establish source-traceability requirements for [research area] in paper/validation/[topic].md"

# Launch all literature reviews for Research Topic 1 together:
Task: "Conduct literature review on [Concept1] in literature-review/[concept1].md"
Task: "Analyze [Concept2] implementations in literature-review/[concept2].md"
```

---

## Research Strategy

### Core First (Research Topic 1 Only)

1. Complete Phase 1: Research Setup
2. Complete Phase 2: Foundational Research (CRITICAL - blocks all topics)
3. Complete Phase 3: Research Topic 1
4. **STOP and VALIDATE**: Verify Research Topic 1 independently
5. Document preliminary findings if ready

### Incremental Delivery

1. Complete Setup + Foundational → Research foundation ready
2. Add Research Topic 1 → Validate independently → Document (Core contribution!)
3. Add Research Topic 2 → Validate independently → Document
4. Add Research Topic 3 → Validate independently → Document
5. Each topic adds value without invalidating previous topics

### Parallel Team Strategy

With multiple researchers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Researcher A: Research Topic 1
   - Researcher B: Research Topic 2
   - Researcher C: Research Topic 3
3. Topics complete and synthesize independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Topic] label maps task to specific research topic for traceability
- Each research topic should be independently completable and validatable
- Verify validation criteria are defined before research begins
- Commit after each task or logical group
- Stop at any checkpoint to validate topic independently
- Avoid: vague tasks, same file conflicts, cross-topic dependencies that break independence
- Ensure all work complies with Physical AI & Humanoid Robotics Research Paper Constitution
