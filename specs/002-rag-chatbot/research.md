# Research Summary: Integrated RAG Chatbot for Physical AI & Humanoid Robotics Textbook

## Decision: Content Chunking Strategy
**Rationale**: For effective RAG, content needs to be chunked at semantic boundaries to maintain context while fitting embedding model limits.
**Alternatives considered**:
- Fixed-size token chunks (simpler but may break semantic context)
- Recursive splitting by characters (may not respect document structure)
- Semantic splitting based on meaning (more complex but preserves context)
**Chosen approach**: Split by document sections (headings, paragraphs) with overlap to preserve context between chunks.

## Decision: Qwen Embedding Model Selection
**Rationale**: Qwen embedding models provide high-quality embeddings with good performance for the free-tier architecture.
**Alternatives considered**:
- Other embedding models available through OpenRouter
- Open-source embedding models (requires self-hosting)
- Commercial embedding models (may exceed free-tier budget)
**Chosen approach**: Use Qwen embedding model available through OpenRouter for consistency with the specified tech stack.

## Decision: FastAPI Backend Architecture
**Rationale**: FastAPI provides async support, automatic API documentation, and good performance for the RAG pipeline.
**Alternatives considered**:
- Flask (simpler but less performant for async operations)
- Django (more complex than needed for API-only service)
- Node.js/Express (different language ecosystem)
**Chosen approach**: FastAPI with async endpoints for optimal performance with LLM API calls.

## Decision: Agent Framework Selection
**Rationale**: The specification mentions OpenAI Agents / ChatKit SDK as the agent framework.
**Alternatives considered**:
- LangChain (mature ecosystem, but not specified in requirements)
- CrewAI (agent collaboration features)
- Custom agent implementation (full control but more work)
**Chosen approach**: Use OpenAI Agents SDK as specified in the technology stack.

## Decision: Frontend Integration Approach
**Rationale**: Docusaurus allows custom React components to be injected into the theme.
**Alternatives considered**:
- Separate iframe integration (more isolated but less seamless)
- JavaScript injection (less maintainable)
- Plugin-based integration (more standard for Docusaurus)
**Chosen approach**: React components integrated as Docusaurus theme components for seamless user experience.

## Decision: Deployment Strategy
**Rationale**: Free-tier services needed to meet the specified constraints.
**Alternatives considered**:
- Self-hosted solutions (more control but higher cost)
- Different cloud providers (may exceed free-tier limits)
- Serverless functions (may have cold start issues)
**Chosen approach**: Deploy backend on Railway/Fly.io/Render (as specified) with Qdrant Cloud and Neon Postgres.

## Decision: Selected Text Mode Implementation
**Rationale**: Need to ensure strict adherence to using only selected text without hallucination.
**Alternatives considered**:
- Augmenting selected text with context from surrounding content
- Strictly limiting to only selected text (as specified)
- Hybrid approach with user control
**Chosen approach**: Strictly limit to selected text only to prevent hallucination as specified in requirements.