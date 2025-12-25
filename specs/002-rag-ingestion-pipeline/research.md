# Research: RAG Data Ingestion & Vector Storage Pipeline

**Feature**: 002-rag-ingestion-pipeline
**Date**: December 25, 2025

## Overview

This research document addresses the technical requirements and decisions for implementing the RAG data ingestion pipeline. It covers URL ingestion, text chunking, embedding generation with Cohere, and vector storage in Qdrant.

## Decision: URL Content Extraction Method
**Rationale**: Need to extract clean text content from HTML pages while removing navigation, ads, and other non-content elements.
**Decision**: Use BeautifulSoup4 with newspaper3k or jina-ai reader for optimal content extraction.
**Alternatives considered**:
- requests + regex (insufficient for complex HTML)
- scrapy (overkill for simple extraction)
- newspaper3k alone (good but limited customization)
- jina-ai reader (good for clean extraction)

## Decision: Text Chunking Strategy
**Rationale**: Need to split documents into semantically meaningful chunks with overlap to preserve context.
**Decision**: Use recursive character text splitter with configurable chunk size (default 512 tokens) and 20% overlap.
**Alternatives considered**:
- Sentence-based splitting (may create uneven chunks)
- Fixed character length (may break semantic meaning)
- Semantic splitting based on meaning (more complex, potential API cost)

## Decision: Embedding Model Selection
**Rationale**: Need to use Cohere embedding models as specified in requirements.
**Decision**: Use Cohere's embed-multilingual-v3.0 model for its multilingual support and efficiency.
**Alternatives considered**:
- embed-english-v3.0 (limited to English content)
- Older Cohere models (less efficient than v3)

## Decision: Qdrant Vector Storage Configuration
**Rationale**: Need to store embeddings with metadata in Qdrant for efficient similarity search.
**Decision**: Use cosine similarity with 1024-dimensional vectors (Cohere's output size), with payload indexing for metadata filtering.
**Alternatives considered**:
- Different similarity metrics (cosine is standard for embeddings)
- Different vector dimensions (must match embedding model output)

## Decision: API Framework
**Rationale**: Need a modern Python web framework for the backend service.
**Decision**: FastAPI for its async support, automatic API documentation, and performance.
**Alternatives considered**:
- Flask (less performant, more manual work)
- Django (overkill for API-only service)

## Decision: Environment Management
**Rationale**: Need secure management of API keys and configuration.
**Decision**: Use python-dotenv with .env files and proper .gitignore for secrets.
**Alternatives considered**:
- Hardcoded values (insecure)
- Environment variables only (less convenient for local development)

## Technical Unknowns Resolved

1. **URL timeout handling**: Implement configurable timeout values (default 30 seconds) with retry logic
2. **Memory management for large documents**: Process documents in chunks to avoid memory issues
3. **Error handling strategy**: Comprehensive error logging with graceful degradation
4. **Batch processing approach**: Queue-based processing for handling multiple URLs efficiently