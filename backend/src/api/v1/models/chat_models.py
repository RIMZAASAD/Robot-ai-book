"""
Chat-related data models for the Full-Stack Integration feature.

This module contains Pydantic models for chat requests and responses
for the frontend-backend communication.
"""

from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from uuid import uuid4
from datetime import datetime


class ChatRequest(BaseModel):
    """Request model for the chat endpoint"""

    query: str = Field(..., min_length=1, max_length=1000)
    include_citations: bool = Field(default=True)
    session_id: Optional[str] = Field(default=None)

    class Config:
        json_schema_extra = {
            "example": {
                "query": "What is artificial intelligence?",
                "include_citations": True,
                "session_id": "sess_1234567890"
            }
        }


class Citation(BaseModel):
    """Model for source citations in chat responses"""

    source_url: str
    document_id: str
    similarity_score: float = Field(ge=0.0, le=1.0)
    chunk_index: Optional[int] = None
    content_preview: Optional[str] = None

    class Config:
        json_schema_extra = {
            "example": {
                "source_url": "https://example.com/ai-textbook",
                "document_id": "doc_1234567890",
                "similarity_score": 0.87,
                "chunk_index": 2,
                "content_preview": "Artificial intelligence (AI) is intelligence demonstrated by machines..."
            }
        }


class ChatResponse(BaseModel):
    """Response model for the chat endpoint"""

    response_id: str = Field(default_factory=lambda: f"resp_{uuid4().hex}")
    response: str
    citations: List[Citation] = Field(default=[])
    retrieved_chunks_count: int
    processing_time_ms: int
    confidence: float = Field(ge=0.0, le=1.0)
    session_id: str
    timestamp: datetime = Field(default_factory=datetime.now)

    class Config:
        json_schema_extra = {
            "example": {
                "response_id": "resp_1234567890",
                "response": "Artificial intelligence (AI) is intelligence demonstrated by machines...",
                "citations": [
                    {
                        "source_url": "https://example.com/ai-textbook",
                        "document_id": "doc_1234567890",
                        "similarity_score": 0.87,
                        "chunk_index": 2,
                        "content_preview": "Artificial intelligence (AI) is intelligence demonstrated by machines..."
                    }
                ],
                "retrieved_chunks_count": 3,
                "processing_time_ms": 1250,
                "confidence": 0.92,
                "session_id": "sess_1234567890"
            }
        }


class ErrorApiResponse(BaseModel):
    """Error response model for API errors"""

    error: str
    message: str
    details: Optional[Dict[str, Any]] = None

    class Config:
        json_schema_extra = {
            "example": {
                "error": "QUERY_PROCESSING_ERROR",
                "message": "Error processing the query",
                "details": {"reason": "Backend service unavailable"}
            }
        }


class HealthResponse(BaseModel):
    """Response model for health check endpoint"""

    status: str
    service: str
    timestamp: str
    dependencies: Optional[dict] = None

    class Config:
        json_schema_extra = {
            "example": {
                "status": "healthy",
                "service": "Chat Service",
                "timestamp": "2025-12-26T10:00:00Z",
                "dependencies": {
                    "openai": "healthy",
                    "qdrant": "healthy",
                    "cohere": "healthy"
                }
            }
        }


class ChatStatusResponse(BaseModel):
    """Response model for chat status endpoint"""

    status: str
    total_queries: int
    successful_queries: int
    success_rate: float
    avg_response_time_ms: int
    last_updated: str

    class Config:
        json_schema_extra = {
            "example": {
                "status": "running",
                "total_queries": 1250,
                "successful_queries": 1245,
                "success_rate": 99.6,
                "avg_response_time_ms": 1150,
                "last_updated": "2025-12-26T10:00:00Z"
            }
        }