# Quickstart: RAG Data Ingestion & Vector Storage Pipeline

**Feature**: 002-rag-ingestion-pipeline
**Date**: December 25, 2025

## Prerequisites

- Python 3.11 or higher
- uv (Python package manager)
- Cohere API key
- Qdrant instance (cloud or local)

## Setup

### 1. Clone and Initialize the Project

```bash
# Create the backend directory
mkdir backend
cd backend

# Initialize the project with uv
uv init
```

### 2. Install Dependencies

Create a `requirements.txt` file with the following content:

```txt
fastapi==0.104.1
uvicorn==0.24.0
python-dotenv==1.0.0
cohere==5.5.3
qdrant-client==1.8.0
beautifulsoup4==4.12.2
requests==2.31.0
httpx==0.25.2
newspaper3k==0.2.8
pytest==7.4.3
pydantic==2.5.0
```

Install the dependencies:

```bash
uv pip install -r requirements.txt
```

### 3. Configure Environment Variables

Create a `.env` file with the following content:

```env
# Cohere Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Configuration
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_HOST=localhost
QDRANT_PORT=6333

# Application Configuration
HOST=0.0.0.0
PORT=8000
CHUNK_SIZE=512
CHUNK_OVERLAP=102  # 20% of chunk size
```

Create an example file `.env.example` with the same structure but empty values.

### 4. Project Structure

Create the following simplified directory structure:

```bash
backend/
├── main.py              # Single file containing all functionality
├── tests/
│   ├── __init__.py
│   ├── unit/
│   ├── integration/
│   └── contract/
├── requirements.txt
├── .env
├── .env.example
├── run_server.py
└── README.md
```

## Running the Service

### 1. Start the Backend Server

```bash
cd backend
uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
```

Or using the run script:

```bash
python run_server.py
```

### 2. Test the API

Once the server is running, you can test the ingestion endpoint:

```bash
curl -X POST "http://localhost:8000/v1/ingest" \
  -H "Content-Type: application/json" \
  -d '{
    "urls": ["https://example.com/page1", "https://example.com/page2"],
    "job_name": "example_ingestion"
  }'
```

### 3. Check the Status of an Ingestion Job

```bash
curl -X GET "http://localhost:8000/v1/ingest/status/{job_id}"
```

## Running Tests

Execute the test suite:

```bash
cd backend
pytest tests/ -v
```

## CLI Usage

The ingestion pipeline also provides a command-line interface:

```bash
python -m src.cli.ingestion_cli --urls https://example.com/page1 https://example.com/page2 --job-name example-job
```

## Configuration Options

- `CHUNK_SIZE`: Size of text chunks in tokens (default: 512)
- `CHUNK_OVERLAP`: Overlap between chunks in tokens (default: 102, 20% of chunk size)
- `HOST` and `PORT`: Server binding configuration
- `COHERE_API_KEY`: API key for Cohere embedding service
- `QDRANT_*`: Connection parameters for Qdrant vector database