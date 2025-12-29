"""
Base agent service class for the RAG Agent Construction feature.

This module provides the base class for agent services with common functionality.
"""

import logging
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional
from datetime import datetime
from ..models.agent_models import UserQuery, RetrievedChunk, AgentResponse, AgentToolCall


class BaseAgentService(ABC):
    """Base class for agent services with common functionality"""

    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)

    @abstractmethod
    async def process_query(self, user_query: UserQuery) -> AgentResponse:
        """Process a user query and return an agent response"""
        pass

    @abstractmethod
    async def validate_response(self, response: AgentResponse, retrieved_chunks: List[RetrievedChunk]) -> bool:
        """Validate that the response is grounded in the retrieved chunks"""
        pass

    def log_agent_operation(self, operation: str, details: Dict[str, Any]):
        """Log agent operations with details"""
        self.logger.info(f"Agent operation: {operation} - Details: {details}")

    def calculate_confidence_score(self, retrieved_chunks: List[RetrievedChunk], response_quality: float) -> float:
        """Calculate confidence score based on retrieved chunks and response quality"""
        if not retrieved_chunks:
            return 0.0

        # Calculate average similarity score
        avg_similarity = sum(chunk.similarity_score for chunk in retrieved_chunks) / len(retrieved_chunks)

        # Combine with response quality
        confidence = (avg_similarity * 0.7 + response_quality * 0.3)
        return min(confidence, 1.0)  # Ensure it doesn't exceed 1.0