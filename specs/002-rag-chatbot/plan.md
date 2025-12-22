# Implementation Plan: Integrated RAG Chatbot for Physical AI & Humanoid Robotics Textbook

**Branch**: `002-rag-chatbot` | **Date**: 2025-12-15 | **Spec**: [link](../specs/002-rag-chatbot/spec.md)
**Input**: Feature specification from `/specs/002-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement an integrated RAG (Retrieval-Augmented Generation) chatbot that allows students to ask questions about Physical AI & Humanoid Robotics textbook content. The system will use Qwen embeddings, Qdrant vector database, and OpenRouter API to provide accurate, contextually relevant answers based on textbook content only, with an additional mode for answering questions based solely on user-selected text.

## Technical Context

**Language/Version**: Python 3.11, JavaScript/TypeScript for frontend integration
**Primary Dependencies**: FastAPI, OpenAI Python SDK, Qdrant client, Neon Postgres driver, React for Docusaurus integration
**Storage**: Qdrant Cloud (vector database), Neon Serverless Postgres (metadata), Docusaurus markdown files
**Testing**: pytest for backend, Jest for frontend components
**Target Platform**: Linux server (backend deployment), Web browser (frontend integration)
**Project Type**: Web application with frontend and backend components
**Performance Goals**: <3 second response time for typical queries, handle up to 100 concurrent users
**Constraints**: Free-tier architecture constraints, no external content hallucination in selected-text mode
**Scale/Scope**: Support for all textbook content, multiple concurrent users, integration with Docusaurus theme

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The implementation must comply with the Physical AI & Humanoid Robotics Textbook Constitution:
- ✅ RAG Stack: Using required stack (OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, Qdrant Cloud Free Tier)
- ✅ Publishing: Integration with Docusaurus as required
- ✅ Technical Standards: Following production-ready code practices
- ✅ Constraints: Working within free-tier architecture limitations

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── chat.py
│   │   ├── document.py
│   │   └── user_session.py
│   ├── services/
│   │   ├── embedding_service.py
│   │   ├── rag_service.py
│   │   ├── qdrant_service.py
│   │   └── postgres_service.py
│   ├── api/
│   │   ├── v1/
│   │   │   ├── chat.py
│   │   │   ├── ingest.py
│   │   │   └── selected_text.py
│   │   └── deps.py
│   ├── agents/
│   │   └── rag_agent.py
│   └── main.py
└── tests/
    ├── unit/
    ├── integration/
    └── contract/

frontend/
├── src/
│   ├── components/
│   │   ├── ChatWidget.jsx
│   │   ├── ChatModal.jsx
│   │   └── SelectedTextButton.jsx
│   ├── hooks/
│   │   └── useChat.js
│   └── services/
│       └── api.js
└── tests/
    └── unit/
```

**Structure Decision**: Web application with separate backend (FastAPI) and frontend (React components for Docusaurus integration) to maintain clear separation of concerns while enabling tight integration with the Docusaurus textbook platform.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [N/A] | [No violations identified] | [Constitution requirements met] |