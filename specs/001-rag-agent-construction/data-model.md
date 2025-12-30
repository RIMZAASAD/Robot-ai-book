# Data Model: RAG Agent Construction

**Feature**: 001-rag-agent-construction
**Created**: 2025-12-26
**Status**: Complete

## Entity Definitions

### UserQuery
**Description**: Natural language input from users seeking information about AI concepts

**Fields**:
- `query_text` (string): The natural language query submitted by the user
- `query_id` (string): Unique identifier for the query (UUID)
- `timestamp` (datetime): When the query was submitted
- `metadata` (object): Additional context about the query (optional)

**Validation Rules**:
- `query_text` must be non-empty
- `query_text` must be less than 1000 characters
- `query_id` must be unique

### RetrievedChunk
**Description**: Document segments returned by the RAG retrieval pipeline with associated metadata

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

### AgentResponse
**Description**: Synthesized answer based on retrieved content with proper citations

**Fields**:
- `response_id` (string): Unique identifier for the response
- `content` (string): The agent's response to the user query
- `source_citations` (array): List of source documents referenced in the response
- `confidence_score` (float): Agent's confidence in the response (0.0-1.0)
- `timestamp` (datetime): When the response was generated
- `query_id` (string): Reference to the original query

**Validation Rules**:
- `content` must be non-empty
- `confidence_score` must be between 0.0 and 1.0
- `source_citations` must contain valid references to retrieved chunks
- Must be linked to a valid `query_id`

### AgentToolCall
**Description**: Record of tools called by the agent during query processing

**Fields**:
- `call_id` (string): Unique identifier for the tool call
- `tool_name` (string): Name of the tool that was called
- `parameters` (object): Parameters passed to the tool
- `result` (object): Result returned by the tool
- `timestamp` (datetime): When the tool was called
- `query_id` (string): Reference to the query that triggered the call

**Validation Rules**:
- `tool_name` must be a registered tool
- `call_id` must be unique
- Must be linked to a valid `query_id`

## Relationships

- `UserQuery` 1 → * `AgentToolCall` (One query can trigger multiple tool calls)
- `UserQuery` 1 → * `RetrievedChunk` (One query can return multiple chunks)
- `RetrievedChunk` * → 1 `AgentResponse` (Multiple chunks inform one response)
- `UserQuery` 1 → 1 `AgentResponse` (One query generates one response)

## State Transitions

### UserQuery State Model
- `PENDING` → Query received, processing started
- `PROCESSING` → Agent is working on the query
- `COMPLETED` → Response generated successfully
- `FAILED` → Error occurred during processing

### AgentResponse State Model
- `DRAFT` → Response is being generated
- `VALIDATED` → Response has been validated against retrieved content
- `READY` → Response is ready for delivery to user
- `DELIVERED` → Response has been sent to the user

## API Models

### AgentQueryRequest
**Purpose**: Request model for the agent endpoint

**Fields**:
- `query` (string): The user's natural language query
- `include_citations` (boolean): Whether to include source citations (default: true)
- `max_chunks` (integer): Maximum number of chunks to retrieve (default: 5)

### AgentQueryResponse
**Purpose**: Response model for the agent endpoint

**Fields**:
- `response` (string): The agent's response to the query
- `citations` (array): List of source citations used in the response
- `retrieved_chunks_count` (integer): Number of chunks retrieved
- `processing_time_ms` (integer): Time taken to process the query
- `confidence` (float): Confidence score for the response