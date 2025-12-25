# Quickstart: RAG Retrieval Pipeline Validation & Testing

**Feature**: 003-rag-retrieval-validation
**Date**: December 25, 2025

## Prerequisites

- Python 3.11 or higher
- uv (Python package manager)
- Cohere API key
- Qdrant instance (cloud or local) with existing content
- Access to the existing RAG ingestion pipeline data

## Setup

### 1. Navigate to the Backend Directory

```bash
cd backend
```

### 2. Install Dependencies

If not already installed as part of the existing RAG pipeline:

```bash
uv pip install -r requirements.txt
```

### 3. Configure Environment Variables

Ensure your `.env` file contains the necessary configuration:

```env
# Cohere Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Configuration
QDRANT_URL=your_qdrant_url_here  # Leave empty for local Qdrant
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_HOST=localhost
QDRANT_PORT=6333

# Application Configuration
HOST=0.0.0.0
PORT=8000
```

### 4. Verify Existing Pipeline Data

Ensure the Qdrant database contains vectors from the ingestion pipeline:

```bash
# Check that the collection exists and has content
python -c "
from qdrant_client import QdrantClient
import os
qdrant_url = os.getenv('QDRANT_URL')
qdrant_api_key = os.getenv('QDRANT_API_KEY')
if qdrant_url and qdrant_api_key:
    client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
else:
    client = QdrantClient(host=os.getenv('QDRANT_HOST', 'localhost'), port=int(os.getenv('QDRANT_PORT', 6333)))
collection_name = 'textbook_content'
collection_info = client.get_collection(collection_name)
print(f'Collection {collection_name} has {collection_info.points_count} vectors')
"
```

## Running the Validation Service

### 1. Start the Validation Service

```bash
cd backend
uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload
```

Or using the run script:

```bash
python run_server.py
```

### 2. Run Validation Tests

Execute the validation pipeline with sample queries:

```bash
python -c "
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath('.')))
from src.validation.retrieval_validator import RetrievalValidator
from src.config.validation_config import ValidationConfig

config = ValidationConfig()
validator = RetrievalValidator(config)

# Run validation with sample queries
sample_queries = [
    'What is artificial intelligence?',
    'Explain machine learning concepts',
    'How does natural language processing work?'
]

results = validator.validate_retrieval(sample_queries)
print('Validation Results:', results)
"
```

### 3. Test the Validation API

Once the server is running, you can test the validation endpoint:

```bash
curl -X POST "http://localhost:8000/v1/validation/run" \
  -H "Content-Type: application/json" \
  -d '{
    \"queries\": [\"What is artificial intelligence?\", \"Explain machine learning\"],
    \"validation_name\": \"sample_validation_run\"
  }'
```

### 4. Check Validation Results

```bash
curl -X GET "http://localhost:8000/v1/validation/report/{report_id}"
```

## Running Validation Tests

Execute the validation test suite:

```bash
cd backend
pytest tests/validation/ -v
```

## Configuration Options

- `COHERE_API_KEY`: API key for Cohere embedding service
- `QDRANT_*`: Connection parameters for Qdrant vector database
- `VALIDATION_TOP_K`: Number of results to retrieve for validation (default: 5)
- `VALIDATION_THRESHOLD`: Minimum similarity score for relevance (default: 0.7)
- `VALIDATION_TIMEOUT`: Timeout for validation operations in seconds (default: 30)

## Sample Validation Workflow

1. Submit queries to the validation service
2. Generate embeddings for each query using Cohere
3. Search the Qdrant database for similar chunks
4. Analyze retrieved results for relevance and metadata accuracy
5. Generate validation report with metrics
6. Verify results meet success criteria (precision, latency, etc.)