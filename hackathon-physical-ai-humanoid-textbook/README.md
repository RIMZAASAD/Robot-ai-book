# Physical AI & Humanoid Robotics Textbook - RAG Chatbot

This project integrates a Retrieval-Augmented Generation (RAG) chatbot with the Physical AI & Humanoid Robotics textbook content, providing an interactive learning experience.

## Project Structure

- `backend/` - FastAPI backend with RAG functionality, OpenRouter integration, and Qdrant vector database
- `website/` - Docusaurus frontend with integrated chatbot widget
- `start_project.ps1` - PowerShell script to start both services
- `start_project.bat` - Batch script to start both services

## Running the Project

### Quick Start (Recommended)

Use one of the automation scripts to clean, build, and start both services:

**PowerShell (Windows):**
```powershell
./start_project.ps1
```

**Command Prompt (Windows):**
```cmd
start_project.bat
```

These scripts will:
- Stop any existing processes
- Clear Docusaurus cache
- Start the backend API server on port 8000
- Start the frontend website on port 3000
- Open separate windows for each service

### Manual Start

If you prefer to start services manually:

To run the backend API server:
```bash
cd backend
python start_api.py
```

To run the frontend Docusaurus website:
```bash
cd website
npx docusaurus start
```

## Services

- **Backend API**: Runs on `http://localhost:8000`
  - Health check: `http://localhost:8000/health`
  - Chat endpoint: `http://localhost:8000/v1/chat`
  - Selected text endpoint: `http://localhost:8000/v1/chat-selected`

- **Frontend Website**: Runs on `http://localhost:3000`
  - Interactive textbook with integrated chatbot widget
  - Chatbot appears as a floating button in the bottom-right corner

## Features

- **RAG Chatbot**: Ask questions about the textbook content
- **Text Selection Mode**: Select text on any page and ask questions about only that selected text
- **Confidence Scoring**: Responses include confidence scores
- **Citations**: Responses include source citations
- **Session Management**: Conversations are maintained in sessions

## Prerequisites

- Python 3.11+
- Node.js 18+
- OpenRouter API key
- Qdrant Cloud account (or local instance)
- Windows OS (for the provided automation scripts)

## Setup

1. **Backend Setup:**
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

2. **Frontend Setup:**
   ```bash
   cd website
   npm install
   ```

3. **Environment Variables:**
   - Set up your `.env` file in the backend directory with required API keys
   - See `backend/README.md` for environment variable details

## Architecture

The project follows a modern web application architecture:
- **Backend**: FastAPI application with RAG capabilities
- **Vector Database**: Qdrant for content storage and retrieval
- **LLM Service**: OpenRouter for language model inference
- **Frontend**: Docusaurus with React-based chatbot widget
- **Integration**: Seamless communication between frontend and backend

For more detailed information about the backend, see `backend/README.md`.