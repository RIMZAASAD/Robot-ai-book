# RAG Retrieval Pipeline Validation & Testing

This backend service provides validation and testing capabilities for the RAG (Retrieval Augmented Generation) retrieval pipeline. It queries the Qdrant vector database with sample user questions, generates query embeddings using the Cohere model, retrieves relevant document chunks, and validates semantic relevance and metadata accuracy.

## Features

- Query validation with semantic similarity scoring
- Embedding generation consistency validation
- Metadata preservation verification
- Performance monitoring and metrics collection
- Comprehensive validation reporting
- RESTful API for validation operations
- Command-line interface for batch validation

## Architecture

The validation system consists of several components:

- **Validation Models**: Pydantic models for queries, embeddings, chunks, and validation results
- **Validation Services**: Core validation logic for query processing, result analysis, and metadata validation
- **API Endpoints**: REST API for running validations and retrieving reports
- **Configuration**: Settings for validation parameters and thresholds
- **Monitoring**: Performance metrics and logging

## Prerequisites

- Python 3.11+
- uv (Python package manager)
- Cohere API key
- Qdrant instance (cloud or local) with existing content
- Access to the existing RAG ingestion pipeline data

## Setup

1. Install dependencies:
```bash
uv pip install -r requirements.txt
```

2. Configure environment variables:
```bash
cp .env.example .env
# Edit .env with your API keys and configuration
```

## Usage

### As a Web Service

```bash
python run_server.py
```

The API will be available at `http://localhost:8000`

### Command Line Interface

```bash
python -m src.validation.validation_cli --queries "What is artificial intelligence?" "Explain machine learning" --validation-name "sample_validation"
```

### API Endpoints

POST to `/v1/validation/run` with JSON payload:
```json
{
  "queries": ["What is artificial intelligence?", "Explain machine learning"],
  "validation_name": "sample_validation_run",
  "top_k": 5,
  "similarity_threshold": 0.7
}
```

GET to `/v1/validation/report/{report_id}` to retrieve validation results

## Configuration

- `COHERE_API_KEY`: API key for Cohere embedding service
- `QDRANT_*`: Connection parameters for Qdrant vector database
- `VALIDATION_TOP_K`: Number of results to retrieve for validation (default: 5)
- `VALIDATION_THRESHOLD`: Minimum similarity score for relevance (default: 0.7)
- `VALIDATION_TIMEOUT`: Timeout for validation operations in seconds (default: 30)

## Validation Metrics

The system tracks several important metrics:

- **Precision Score**: Percentage of retrieved chunks that are semantically relevant
- **Metadata Accuracy**: Percentage of chunks with complete and valid metadata
- **Average Latency**: Average time to process a query and retrieve results
- **Determinism Score**: Consistency of results across multiple runs
- **Relevance Score**: Average similarity score of retrieved chunks

## Testing

Run the validation test suite:

```bash
# Unit tests
pytest tests/unit/ -v

# Integration tests
pytest tests/integration/ -v

# Validation-specific tests
pytest tests/validation/ -v
```

## Endpoints

- `POST /v1/validation/run` - Run validation on provided queries
- `GET /v1/validation/report/{report_id}` - Get validation report
- `GET /v1/validation/health` - Health check for validation service
- `GET /v1/validation/test` - Test endpoint for validation functionality

## Validation Process

1. Submit queries to the validation service
2. Generate embeddings for each query using Cohere
3. Search the Qdrant database for similar chunks
4. Analyze retrieved results for relevance and metadata accuracy
5. Generate validation report with metrics
6. Verify results meet success criteria (precision, latency, etc.)