# Data Model: RAG Data Ingestion & Vector Storage Pipeline

**Feature**: 002-rag-ingestion-pipeline
**Date**: December 25, 2025

## Entity: Document
**Description**: Represents the original content source with URL, title, and metadata attributes

**Fields**:
- `id` (string): Unique identifier for the document
- `url` (string): Source URL of the document
- `title` (string): Title of the document
- `content` (string): Full extracted content from the URL
- `created_at` (datetime): Timestamp when document was ingested
- `updated_at` (datetime): Timestamp of last update
- `status` (enum): Processing status (pending, processing, completed, failed)

**Validation rules**:
- URL must be a valid HTTP/HTTPS URL
- Title and content must not be empty
- Status must be one of the defined enum values

## Entity: TextChunk
**Description**: Represents a semantically meaningful segment of extracted text with position context and content

**Fields**:
- `id` (string): Unique identifier for the text chunk
- `document_id` (string): Reference to parent Document
- `content` (string): The actual text content of the chunk
- `chunk_index` (integer): Position of this chunk in the original document
- `chunk_size` (integer): Size of the chunk in characters/tokens
- `overlap_size` (integer): Size of overlap with adjacent chunks
- `created_at` (datetime): Timestamp when chunk was created

**Validation rules**:
- Content must not be empty
- Document_id must reference an existing Document
- Chunk_index must be non-negative
- Chunk_size must be positive

## Entity: EmbeddingVector
**Description**: Represents the semantic vector representation of a text chunk, stored in Qdrant

**Fields**:
- `id` (string): Unique identifier for the embedding vector
- `chunk_id` (string): Reference to parent TextChunk
- `vector` (array of floats): The actual embedding vector values
- `vector_size` (integer): Dimension of the embedding vector
- `model_name` (string): Name of the model used to generate the embedding
- `created_at` (datetime): Timestamp when embedding was generated

**Validation rules**:
- Vector must have consistent dimensions based on the model
- Chunk_id must reference an existing TextChunk
- Vector_size must match the expected size for the model

## Entity: IngestionJob
**Description**: Represents a batch processing task containing multiple URLs to be processed

**Fields**:
- `id` (string): Unique identifier for the ingestion job
- `job_name` (string): Name/description of the job
- `urls` (array of strings): List of URLs to process
- `status` (enum): Job status (pending, running, completed, failed, cancelled)
- `progress` (integer): Percentage of completion (0-100)
- `total_documents` (integer): Total number of documents to process
- `processed_documents` (integer): Number of documents processed
- `created_at` (datetime): Timestamp when job was created
- `updated_at` (datetime): Timestamp of last status update
- `completed_at` (datetime, optional): Timestamp when job was completed

**Validation rules**:
- URLs array must not be empty
- Status must be one of the defined enum values
- Progress must be between 0 and 100
- Total documents must be positive
- Processed documents must not exceed total documents

## Relationships

- **Document** 1 → * **TextChunk**: One document can have multiple text chunks
- **TextChunk** 1 → 1 **EmbeddingVector**: Each text chunk has exactly one embedding vector
- **IngestionJob** 1 → * **Document**: One ingestion job can create multiple documents

## State Transitions

### Document Status Transitions
- `pending` → `processing`: When content extraction begins
- `processing` → `completed`: When content extraction and validation succeed
- `processing` → `failed`: When content extraction encounters an error

### IngestionJob Status Transitions
- `pending` → `running`: When job processing begins
- `running` → `completed`: When all URLs in the job are processed
- `running` → `failed`: When a critical error occurs during processing
- `running` → `cancelled`: When job is manually cancelled