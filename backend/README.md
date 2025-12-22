# RAG Chatbot Backend

Backend API for the Integrated RAG Chatbot for Physical AI & Humanoid Robotics Textbook.

## Features

- FastAPI-based REST API
- RAG (Retrieval-Augmented Generation) functionality
- Integration with OpenRouter for LLM inference
- Qdrant vector database for content storage
- Support for both full-textbook and selected-text Q&A modes

## Prerequisites

- Python 3.11+
- OpenRouter API key
- Qdrant Cloud account (or local instance)
- Textbook content in markdown format

## Setup

1. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Set up environment variables:**
   Create a `.env` file in the backend root with:
   ```env
   OPENROUTER_API_KEY=your_openrouter_api_key
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key  # if using cloud service
   QWEN_EMBEDDING_MODEL=nvidia/llama-3.2-nv-embedqa-1b-v1  # or appropriate embedding model
   ```

3. **Run the application:**
   ```bash
   uvicorn src.main:app --reload --port 8000
   ```

## API Endpoints

### Chat Endpoints

- `POST /v1/chat` - Full-textbook RAG Q&A
- `POST /v1/chat-selected` - Selected-text-only Q&A

### Ingestion Endpoints

- `POST /v1/ingest` - Ingest specific content with metadata
- `POST /v1/ingest-textbook` - Ingest entire textbook from markdown files

## Environment Variables

- `OPENROUTER_API_KEY`: Your OpenRouter API key
- `QDRANT_URL`: URL to your Qdrant instance
- `QDRANT_API_KEY`: API key for Qdrant (if using cloud service)
- `QWEN_EMBEDDING_MODEL`: Model to use for embeddings (default: nvidia/llama-3.2-nv-embedqa-1b-v1)

## Testing

Run the basic implementation test:
```bash
python test_implementation.py
```

## Architecture

- `src/main.py` - FastAPI application entry point
- `src/models/` - Pydantic models for request/response validation
- `src/services/` - Core services (embedding, Qdrant, OpenRouter, content ingestion)
- `src/api/v1/` - API route definitions
- `src/agents/` - Agent logic (to be implemented)

## Development

The application follows a service-oriented architecture with clear separation of concerns:

1. **Services Layer**: Handles external integrations (OpenRouter, Qdrant, embedding models)
2. **Models Layer**: Defines data structures and validation
3. **API Layer**: Defines endpoints and handles request/response logic
4. **Main Application**: Orchestrates the components and handles startup/shutdown

## Deployment

The application is designed for deployment on platforms like Railway, Fly.io, or Render. Make sure to configure the environment variables appropriately for your deployment environment.