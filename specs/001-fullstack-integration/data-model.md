# Data Model: Full-Stack Integration for AI Textbook Chatbot

**Feature**: 001-fullstack-integration
**Created**: 2025-12-26
**Status**: Complete

## Entity Definitions

### UserQuery
**Description**: Natural language input from users seeking information about AI concepts

**Fields**:
- `query_id` (string): Unique identifier for the query (UUID)
- `query_text` (string): The natural language query submitted by the user
- `timestamp` (datetime): When the query was submitted
- `session_id` (string): Session identifier for the current interaction
- `user_id` (string): Identifier for the user (if applicable)

**Validation Rules**:
- `query_text` must be non-empty
- `query_text` must be less than 1000 characters
- `query_id` must be unique per session

### AgentResponse
**Description**: Synthesized answer from the backend RAG agent with proper citations

**Fields**:
- `response_id` (string): Unique identifier for the response
- `content` (string): The agent's response to the user query
- `source_citations` (array): List of source documents referenced in the response
- `confidence_score` (float): Agent's confidence in the response (0.0-1.0)
- `timestamp` (datetime): When the response was generated
- `query_id` (string): Reference to the original query
- `processing_time_ms` (integer): Time taken to process the query

**Validation Rules**:
- `content` must be non-empty
- `confidence_score` must be between 0.0 and 1.0
- `source_citations` must contain valid references to retrieved chunks
- Must be linked to a valid `query_id`

### RetrievedChunk
**Description**: Document segments used to generate the response with source metadata

**Fields**:
- `chunk_id` (string): Unique identifier for the chunk
- `content` (string): The actual text content of the chunk
- `similarity_score` (float): Similarity score from vector search (0.0-1.0)
- `source_url` (string): URL where the original content was sourced from
- `document_id` (string): ID of the original document
- `chunk_index` (integer): Position of the chunk in the original document
- `metadata` (object): Additional metadata associated with the chunk

**Validation Rules**:
- `content` must be non-empty
- `similarity_score` must be between 0.0 and 1.0
- `source_url` must be a valid URL format
- `chunk_id` must be unique per query

### APICommunication
**Description**: Request/response objects for frontend-backend communication

**Fields**:
- `request_id` (string): Unique identifier for the API request
- `endpoint` (string): The API endpoint being called
- `method` (string): HTTP method (GET, POST, etc.)
- `request_payload` (object): The data sent in the request
- `response_payload` (object): The data received in the response
- `status_code` (integer): HTTP status code
- `timestamp` (datetime): When the communication occurred
- `duration_ms` (integer): Time taken for the request

**Validation Rules**:
- `request_id` must be unique
- `method` must be a valid HTTP method
- `status_code` must be a valid HTTP status code

### ErrorState
**Description**: Information about API errors and their handling for user display

**Fields**:
- `error_id` (string): Unique identifier for the error
- `error_type` (string): Type of error (network, validation, server, etc.)
- `error_message` (string): Technical error message
- `user_message` (string): User-friendly error message
- `timestamp` (datetime): When the error occurred
- `request_id` (string): Reference to the request that caused the error
- `retry_count` (integer): Number of times the request was retried

**Validation Rules**:
- `error_id` must be unique
- `error_type` must be a recognized error type
- `user_message` must be non-empty

## Relationships

- `UserQuery` 1 → 1 `AgentResponse` (One query generates one response)
- `UserQuery` 1 → * `RetrievedChunk` (One query can return multiple chunks)
- `AgentResponse` 1 → * `RetrievedChunk` (One response can reference multiple chunks)
- `APICommunication` 1 → 1 `UserQuery` (One API call for one query)
- `ErrorState` 0..1 → 1 `APICommunication` (Error may or may not occur during API communication)

## State Transitions

### UserQuery State Model
- `DRAFT` → Query is being composed
- `SUBMITTED` → Query sent to backend
- `PROCESSING` → Backend is working on the query
- `COMPLETED` → Response received successfully
- `FAILED` → Error occurred during processing

### APICommunication State Model
- `INITIATED` → Request is being prepared
- `SENT` → Request sent to server
- `RECEIVING` → Response being received
- `COMPLETED` → Request completed successfully
- `FAILED` → Request failed with error

### ChatSession State Model
- `IDLE` → No interaction occurring
- `AWAITING_RESPONSE` → Waiting for backend response
- `DISPLAYING_RESPONSE` → Response being shown to user
- `ERROR` → Error state in the session

## API Models

### ChatRequest
**Purpose**: Request model for the chat endpoint

**Fields**:
- `query` (string): The user's natural language query
- `include_citations` (boolean): Whether to include source citations (default: true)
- `session_id` (string): Session identifier for the interaction

### ChatResponse
**Purpose**: Response model for the chat endpoint

**Fields**:
- `response_id` (string): Unique identifier for the response
- `response` (string): The agent's response to the query
- `citations` (array): List of source citations used in the response
- `retrieved_chunks_count` (integer): Number of chunks retrieved
- `processing_time_ms` (integer): Time taken to process the query
- `confidence` (float): Confidence score for the response
- `session_id` (string): Session identifier for the interaction

### ErrorApiResponse
**Purpose**: Error response model for API errors

**Fields**:
- `error` (string): Error code
- `message` (string): User-friendly error message
- `details` (object): Additional error details (optional)