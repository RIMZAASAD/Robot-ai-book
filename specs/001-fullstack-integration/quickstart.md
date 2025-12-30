# Quickstart Guide: Full-Stack Integration for AI Textbook Chatbot

**Feature**: 001-fullstack-integration
**Created**: 2025-12-26

## Overview

This guide provides quick instructions for setting up and using the full-stack integration between the React/Next.js frontend and FastAPI backend for the AI textbook chatbot with animated UX components.

## Prerequisites

- Node.js 18+ (for frontend)
- Python 3.11+ (for backend)
- OpenAI API key
- Cohere API key
- Qdrant vector database with indexed content
- Git for version control

## Backend Setup

### 1. Environment Variables

Create a `.env` file in the backend directory with the following variables:

```bash
OPENAI_API_KEY=your_openai_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=your_collection_name
```

### 2. Backend Installation

```bash
cd backend
pip install -r requirements.txt
```

### 3. Start Backend Server

```bash
cd backend
uvicorn main:app --reload --port 8000
```

## Frontend Setup

### 1. Environment Variables

Create a `.env` file in the frontend directory with the following variables:

```bash
NEXT_PUBLIC_API_BASE_URL=http://localhost:8000
NEXT_PUBLIC_BACKEND_URL=http://localhost:8000
```

### 2. Frontend Installation

```bash
cd frontend
npm install
```

### 3. Start Frontend Development Server

```bash
cd frontend
npm run dev
```

## Usage

### 1. Access the Chat Interface

Open your browser and navigate to `http://localhost:3000` to access the chat interface.

### 2. Submit a Query

1. Type your question about AI concepts in the input field
2. Click the submit button or press Enter
3. The query will be sent to the backend RAG agent
4. The response will be displayed with animated transitions
5. Source metadata will be shown alongside the response

### 3. API Endpoints

The following API endpoints are available:

- `POST /v1/chat/query` - Submit a query to the RAG agent
- `GET /v1/chat/health` - Check chat service health
- `GET /v1/chat/status` - Get chat status and statistics

### 4. Example API Request

```bash
curl -X POST "http://localhost:8000/v1/chat/query" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is artificial intelligence?",
    "include_citations": true,
    "session_id": "session-123"
  }'
```

## Key Components

### Frontend Architecture
- `ChatInput`: Component for user query input with validation
- `ResponseDisplay`: Component for displaying agent responses with metadata
- `LoadingAnimation`: Animated loading indicators during processing
- `ErrorDisplay`: Component for displaying user-friendly error messages
- `useChat`: Custom hook for managing chat state and interactions
- `useApi`: Custom hook for API communication

### Backend Architecture
- `ChatAPI`: FastAPI endpoints for chat functionality
- `RAGAgent`: Agent that processes queries using the RAG pipeline
- `ResponseProcessor`: Service for formatting responses with metadata
- `API Models`: Pydantic models for request/response validation

## Development

### Running Tests

#### Frontend Tests
```bash
cd frontend
npm test
```

#### Backend Tests
```bash
cd backend
pytest tests/integration/test_fullstack_integration.py
```

### Adding New Features

To add new features to the chat interface:

1. Create new components in the `frontend/src/components/ChatInterface` directory
2. Update the data models if needed
3. Add new API endpoints in the backend if needed
4. Update the API contracts
5. Add tests for new functionality

## API Endpoints

- `POST /v1/chat/query` - Submit a query to the RAG agent
- `GET /v1/chat/health` - Check chat service health
- `GET /v1/chat/status` - Get chat status and statistics

## Troubleshooting

### Common Issues

1. **API Connection Errors**: Ensure backend server is running on port 8000
2. **CORS Issues**: Check backend CORS configuration
3. **Animation Performance**: Ensure browser supports CSS transforms and animations
4. **API Keys Missing**: Verify all required API keys are in environment variables

### Logging
- Frontend: Console logs for UI interactions and API calls
- Backend: Structured logs for API requests, responses, and errors
- Check both frontend and backend logs for debugging information

## Animation Features

The chat interface includes several animated UX components:
- Smooth transitions when responses appear
- Loading animations during query processing
- Error state animations for better user feedback
- Interactive transitions for query submission
- Animated metadata display for source citations