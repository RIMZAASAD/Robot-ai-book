# Physical AI & Humanoid Robotics Textbook - RAG Chatbot

This project integrates a Retrieval-Augmented Generation (RAG) chatbot with the Physical AI & Humanoid Robotics textbook content, providing an interactive learning experience.

## Project Structure

- `backend/` - FastAPI backend with RAG functionality, OpenRouter integration, and Qdrant vector database
- `website/` - Docusaurus frontend with integrated chatbot widget
- `frontend/` - Next.js frontend for the Full-Stack Integration feature
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
- Start the Docusaurus frontend website on port 3000
- Open separate windows for each service

### Manual Start

If you prefer to start services manually:

**Option 1: Docusaurus Frontend (Existing Setup)**
To run the backend API server:
```bash
cd backend
python start_api.py
```

To run the Docusaurus frontend website:
```bash
cd website
npx docusaurus start
```

**Option 2: Next.js Frontend (Full-Stack Integration)**
To run the backend API server:
```bash
cd backend
python start_api.py
```

To run the Next.js frontend:
```bash
cd frontend
npm run dev
```

### Full-Stack Integration Frontend

The project now includes a new Next.js frontend for the Full-Stack Integration feature with a dedicated chat interface. This provides an alternative to the Docusaurus-based frontend with more focused chat functionality.

To run the Full-Stack Integration frontend:
1. Start the backend server in one terminal:
   ```bash
   cd backend
   python start_api.py
   ```

2. In another terminal, start the Next.js frontend:
   ```bash
   cd frontend
   npm run dev
   ```

3. Open your browser to `http://localhost:3000` to access the chat interface.

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

## Deployment

The project is configured for Vercel deployment:
- **Vercel Configuration**: `vercel.json` with proper Docusaurus SPA routing
- **Framework Detection**: Uses Vercel's built-in Docusaurus support
- **Build Settings**: Automatically builds and deploys the Docusaurus site

### Deploy to Vercel

To deploy this project to Vercel:

1. **Using Vercel CLI:**
   ```bash
   # Install Vercel CLI globally
   npm install -g vercel

   # Deploy to production
   vercel --prod
   ```

2. **Using Vercel Dashboard:**
   - Push your code to GitHub
   - Go to [Vercel Dashboard](https://vercel.com/dashboard)
   - Import your GitHub repository
   - Vercel will automatically detect the Docusaurus project and use the `vercel.json` configuration
   - The project will build and deploy automatically

The website will be deployed with the configuration in `website/docusaurus.config.js` which is set up for Vercel deployment.

## Development Scripts

Two automation scripts are provided for Windows environments:
- `start_project.ps1` - PowerShell script for Windows
- `start_project.bat` - Batch script for Command Prompt

For other environments, use the manual start process described above.

For more detailed information about the backend, see `backend/README.md`.

## Serving the Website and Root API 🔧

- A convenience `api.py` is available at the project root that re-uses the backend FastAPI app, enables CORS for local development, and can serve the built Docusaurus site from `website/build/` when present.
- To build the Docusaurus site locally, run:

```bash
cd website
npm ci
npm run build
```

or use the helper Python script:

```bash
python scripts/build_website.py
```

- Start the combined API (serves both API and static site) with:

```bash
uvicorn api:app --reload --host 0.0.0.0 --port 8000
```

Then open `http://localhost:8000/` to view the built site and the chatbot widget will call `/v1/chat/query` on the same host.