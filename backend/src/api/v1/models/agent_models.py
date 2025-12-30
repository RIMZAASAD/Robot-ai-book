"""
API models for the RAG Agent Construction feature.

This module contains Pydantic models for API requests and responses
for the agent endpoints.
"""

from pydantic import BaseModel, Field
from typing import List, Optional
from ....models.agent_models import AgentResponse


class AgentQueryRequest(BaseModel):
    """Request model for the agent endpoint"""

    query: str = Field(..., min_length=1, max_length=1000)
    include_citations: bool = Field(default=True)
    max_chunks: int = Field(default=5, ge=1, le=20)
    similarity_threshold: float = Field(default=0.7, ge=0.0, le=1.0)

    class Config:
        json_schema_extra = {
            "example": {
                "query": "What is artificial intelligence?",
                "include_citations": True,
                "max_chunks": 5,
                "similarity_threshold": 0.7
            }
        }


class AgentQueryResponse(BaseModel):
    """Response model for the agent endpoint"""

    response_id: str
    response: str
    citations: List[dict]
    retrieved_chunks_count: int
    processing_time_ms: int
    confidence: float

    class Config:
        json_schema_extra = {
            "example": {
                "response_id": "resp_1234567890",
                "response": "Artificial intelligence (AI) is intelligence demonstrated by machines...",
                "citations": [
                    {
                        "source_url": "https://example.com/ai-textbook",
                        "document_id": "doc_123",
                        "similarity_score": 0.87
                    }
                ],
                "retrieved_chunks_count": 3,
                "processing_time_ms": 1250,
                "confidence": 0.92
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
                "service": "RAG Agent Service",
                "timestamp": "2025-12-26T10:00:00Z",
                "dependencies": {
                    "openai": "healthy",
                    "qdrant": "healthy",
                    "cohere": "healthy"
                }
            }
        }


class AgentStatusResponse(BaseModel):
    """Response model for agent status endpoint"""

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