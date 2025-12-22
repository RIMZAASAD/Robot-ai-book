# Quickstart Guide: Integrated RAG Chatbot for Physical AI & Humanoid Robotics Textbook

## Overview
This guide provides instructions for setting up and running the RAG chatbot system that integrates with the Physical AI & Humanoid Robotics textbook.

## Prerequisites
- Python 3.11+
- Node.js 18+ (for Docusaurus development)
- Access to OpenRouter API
- Qdrant Cloud account (Free Tier)
- Neon Serverless Postgres account

## Backend Setup

### 1. Environment Configuration
```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install fastapi uvicorn python-dotenv openai qdrant-client psycopg2-binary
```

### 2. Environment Variables
Create a `.env` file with the following:
```env
OPENROUTER_API_KEY=your_openrouter_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_postgres_connection_string
QWEN_EMBEDDING_MODEL=qwen/qwen-embedding-model
```

### 3. Run the Backend
```bash
# Start the FastAPI server
uvicorn src.main:app --reload --port 8000
```

## Frontend Integration

### 1. Docusaurus Integration
The chatbot component can be integrated into Docusaurus using the following:

```jsx
// Example integration in Docusaurus
import ChatWidget from '../components/ChatWidget';

// Add to your layout or page
<ChatWidget
  backendUrl="https://your-backend-url.com"
  pageContext={currentPath}
/>
```

### 2. Text Selection Feature
The text selection feature allows users to select content and ask questions about it specifically:

```javascript
// Example text selection handler
function handleTextSelection() {
  const selectedText = window.getSelection().toString();
  if (selectedText) {
    // Show option to ask about selected text
    showAskAboutSelectionButton(selectedText);
  }
}
```

## API Usage Examples

### 1. Ingesting Textbook Content
```bash
curl -X POST http://localhost:8000/v1/ingest \
  -H "Content-Type: application/json" \
  -d '{
    "content": "The ROS (Robot Operating System) is a flexible framework for writing robot software...",
    "title": "Introduction to ROS",
    "chapter": "module-2-ros",
    "section": "chapter-6-ros2-packages",
    "source_file": "docs/module-2-ros/chapter-6-ros2-packages.md"
  }'
```

### 2. Chat with Full Textbook Context
```bash
curl -X POST http://localhost:8000/v1/chat \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is a ROS node?",
    "page_context": "module-2-ros/chapter-6-ros2-packages"
  }'
```

### 3. Chat with Selected Text Only
```bash
curl -X POST http://localhost:8000/v1/chat-selected \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Explain this concept",
    "selected_text": "A ROS node is a process that performs computation. Nodes are the fundamental building blocks of a ROS system..."
  }'
```

## Development Workflow

1. **Content Ingestion**: Run the ingestion process to index all textbook content
2. **Testing**: Use the API endpoints to test chat functionality
3. **Frontend Integration**: Add the chat component to Docusaurus pages
4. **Deployment**: Deploy backend to Railway/Fly.io and integrate with Docusaurus site

## Testing
```bash
# Run backend tests
python -m pytest tests/

# Test the API manually using the examples above
```

## Troubleshooting

- **Embedding issues**: Ensure Qwen embedding model is correctly configured
- **Response time**: Check OpenRouter API key and rate limits
- **Citation accuracy**: Verify content chunking and indexing process