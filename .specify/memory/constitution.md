<!-- Sync Impact Report -->
<!--
Version change: 1.0.0 (assumed initial) → 1.1.0
Modified principles:
- PRINCIPLE_1_NAME → VLA Convergence Mandate
- PRINCIPLE_2_NAME → Anthropomorphic Focus
- PRINCIPLE_3_NAME → Sim-to-Real Rigor
- PRINCIPLE_4_NAME → Real-Time Validation
Added sections:
- Project Overview
- Key Standards
- Constraints
- Success Criteria
Removed sections:
- PRINCIPLE_5_NAME and PRINCIPLE_6_NAME placeholders and descriptions
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/sp.constitution.md: ✅ updated (this file)
- .specify/templates/phr-template.prompt.md: ⚠ pending
- README.md: ⚠ pending
Follow-up TODOs:
- TODO(RATIFICATION_DATE): If actual ratification date is different from the provided one, update it.
-->
# Physical AI & Humanoid Robotics Textbook: From Simulation to Embodied Intelligence Constitution

## Project Overview
**Version**: v1.1.0 (Physical AI Educational Framework)
**Platform/Tools**: Docusaurus, GitHub Pages, Spec-Kit Plus, Claude Code
**Persona**: Humanoid Robotics Systems Architect
**Scope**: Educational content governance for the 13-week course.

## Core Principles

### I. VLA Convergence Mandate
All content must converge on the Vision-Language-Action (VLA) pipeline. Language is the Primary Control Interface for the humanoid (Whisper + LLM Cognitive Planning).

### II. Anthropomorphic Focus
Emphasize the unique challenges of Bipedal Locomotion and Dexterous Manipulation in human-centered environments (stairs, doors). Avoid generic wheeled robotics patterns.

### III. Sim-to-Real Rigor
Content must facilitate training in high-fidelity simulation (NVIDIA Isaac Sim) and ensure the process of downloading trained weights for deployment to the physical edge device (heston) is fully documented.

### IV. Real-Time Validation
All ROS 2 implementations must account for low-latency control, specifically covering ROS 2 QoS profiles configuration for bipedal balance feedback.

## Key Standards (Technical & Platform)

- **Authoring**: Must be written entirely using Spec-Kit Plus and Claude Code.
- **Publishing**: Format content for Docusaurus. Use Docusaurus Admonitions (note, tip, warning) for highlighting critical insights, not generic Markdown.
- **Visualization**: All complex spatial or architectural concepts (Kinematics, TF frames, VLA flow, ZMP stability) must be illustrated using Mermaid diagrams.
- **Code Validation**: All ROS 2 code must be presented as production-ready, including necessary build system configurations (package.xml, setup.py, CMakeLists.txt).
- **RAG Stack**: The embedded RAG Chatbot must be built using the required stack: OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud Free Tier.


## Constraints (Timeline & Hardware)

- **Duration/Structure**: 13 weeks, organized into 4 sequential modules (Foundations → ROS 2 → Simulation → VLA/Humanoid).
- **Target Edge Hardware**: Content must be deployable and optimized for the NVIDIA Jetson Orin Nano (8GB), Intel RealSense D435i, and ReSpeaker USB Mic Array.
- **Target Dev Hardware**: Content must assume a development environment running Ubuntu 22.04 LTS and an NVIDIA RTX 4070 Ti (12GB) or higher GPU.
- **Bonus Requirements**: The structure must allow for the seamless integration of the four optional +50 point bonus features (Subagents, Auth/Background, Personalization, Urdu Translation).


## Success Criteria

- The complete 13-week course content is generated and formatted correctly for Docusaurus.
- The integrated RAG Chatbot is fully functional, answering queries based on the textbook content.
- The Autonomous Humanoid Capstone is fully documented, demonstrating the end-to-end VLA cycle (Voice Command → LLM Plan → Bipedal Navigation → Vision ID → Manipulation) in simulation.
- All technical code samples validate correctly against the Jetson Orin edge constraints.

## Governance

Constitution supersedes all other practices; Amendments require documentation, approval, migration plan.
All PRs/reviews must verify compliance; Complexity must be justified; Use [GUIDANCE_FILE] for runtime development guidance.

**Version**: v1.1.0 | **Ratified**: 2025-01-20 | **Last Amended**: 2025-12-04

