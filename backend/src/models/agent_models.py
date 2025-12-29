"""
Agent-related data models for the RAG Agent Construction feature.

This module contains Pydantic models for representing user queries,
retrieved chunks, agent responses, and tool calls in the RAG agent system.
"""

from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime
from uuid import uuid4


class UserQuery(BaseModel):
    """Natural language input from users seeking information about AI concepts"""

    query_id: str = Field(default_factory=lambda: f"query_{uuid4().hex}")
    query_text: str = Field(..., min_length=1, max_length=1000)
    timestamp: datetime = Field(default_factory=datetime.now)
    metadata: Optional[Dict[str, Any]] = Field(default={})

    class Config:
        json_schema_extra = {
            "example": {
                "query_text": "What is artificial intelligence?",
                "metadata": {"user_id": "user_123"}
            }
        }


class RetrievedChunk(BaseModel):
    """Document segments returned by the RAG retrieval pipeline with associated metadata"""

    chunk_id: str
    content: str
    similarity_score: float = Field(ge=0.0, le=1.0)
    source_url: str
    document_id: str
    chunk_index: int
    metadata: Dict[str, Any] = Field(default={})

    class Config:
        json_schema_extra = {
            "example": {
                "chunk_id": "chunk_123",
                "content": "Artificial intelligence (AI) is intelligence demonstrated by machines...",
                "similarity_score": 0.87,
                "source_url": "https://example.com/ai-textbook",
                "document_id": "doc_123",
                "chunk_index": 2
            }
        }


class AgentToolCall(BaseModel):
    """Record of tools called by the agent during query processing"""

    call_id: str = Field(default_factory=lambda: f"call_{uuid4().hex}")
    tool_name: str
    parameters: Dict[str, Any] = Field(default={})
    result: Optional[Dict[str, Any]] = Field(default=None)
    timestamp: datetime = Field(default_factory=datetime.now)
    query_id: str

    class Config:
        json_schema_extra = {
            "example": {
                "tool_name": "retrieval_tool",
                "parameters": {"query": "What is AI?"},
                "query_id": "query_123"
            }
        }


class AgentResponse(BaseModel):
    """Synthesized answer based on retrieved content with proper citations"""

    response_id: str = Field(default_factory=lambda: f"resp_{uuid4().hex}")
    content: str
    source_citations: List[Dict[str, Any]] = Field(default=[])
    confidence_score: float = Field(ge=0.0, le=1.0)
    timestamp: datetime = Field(default_factory=datetime.now)
    query_id: str

    class Config:
        json_schema_extra = {
            "example": {
                "content": "Artificial intelligence (AI) is intelligence demonstrated by machines...",
                "source_citations": [
                    {
                        "source_url": "https://example.com/ai-textbook",
                        "document_id": "doc_123",
                        "similarity_score": 0.87
                    }
                ],
                "confidence_score": 0.92,
                "query_id": "query_123"
            }
        }