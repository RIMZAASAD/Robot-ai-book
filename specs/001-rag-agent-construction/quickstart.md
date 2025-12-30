# Quickstart Guide: RAG Agent Construction

**Feature**: 001-rag-agent-construction
**Created**: 2025-12-26

## Overview

This guide provides quick instructions for setting up and using the RAG Agent that answers user queries by retrieving information from the validated RAG pipeline.

## Prerequisites

- Python 3.11+
- OpenAI API key
- Cohere API key
- Qdrant vector database with indexed content
- FastAPI-compatible environment

## Setup

### 1. Environment Variables

Create a `.env` file with the following variables:

```bash
OPENAI_API_KEY=your_openai_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=your_collection_name
```

### 2. Installation

```bash
pip install openai fastapi uvicorn cohere qdrant-client python-dotenv pydantic
```

## Usage

### 1. Start the API Server

```bash
cd backend
uvicorn main:app --reload --port 8000
```

### 2. Query the Agent

Send a POST request to the agent endpoint:

```bash
curl -X POST "http://localhost:8000/v1/agent/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is artificial intelligence?",
    "include_citations": true,
    "max_chunks": 5
  }'
```

### 3. Expected Response

```json
{
  "response_id": "uuid-string",
  "response": "Artificial intelligence (AI) is intelligence demonstrated by machines...",
  "citations": [
    {
      "source_url": "https://example.com/ai-textbook",
      "document_id": "doc-123",
      "similarity_score": 0.87
    }
  ],
  "retrieved_chunks_count": 3,
  "processing_time_ms": 1250,
  "confidence": 0.92
}
```

## Key Components

### Agent Architecture
- `RAGAgent`: Main agent class that processes queries
- `RetrievalTool`: Tool that interfaces with Qdrant for document retrieval
- `AgentService`: Service layer that orchestrates agent operations
- `AgentAPI`: FastAPI endpoints for agent interactions

### Configuration
- Agent behavior is configured through `agent_config.py`
- Retrieval parameters can be adjusted (top_k, similarity threshold)
- API rate limits and security settings are configurable

## Development

### Running Tests

```bash
pytest tests/unit/test_rag_agent.py
pytest tests/integration/test_agent_api.py
```

### Adding New Tools

To add a new tool to the agent:

1. Create a new tool class inheriting from the base tool
2. Register it in the tool registry
3. Update the agent configuration to include the new tool

## API Endpoints

- `POST /v1/agent/query` - Submit a query to the RAG agent
- `GET /v1/agent/health` - Check agent service health
- `GET /v1/agent/status` - Get agent status and statistics