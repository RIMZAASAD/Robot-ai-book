"""
Configuration settings for the RAG Agent Construction feature.

This module contains configuration parameters and settings for the agent system.
"""

import logging
from typing import Optional
from pydantic import Field
from .settings import Settings


class AgentSettings(Settings):
    """Configuration settings for the RAG agent"""

    # Agent-specific settings
    agent_model: str = Field(default="gpt-4-turbo-preview", description="OpenAI model to use for the agent")
    agent_temperature: float = Field(default=0.1, ge=0.0, le=1.0, description="Temperature for agent responses")
    agent_max_tokens: int = Field(default=1000, ge=100, le=4000, description="Max tokens for agent responses")

    # Retrieval settings
    retrieval_top_k: int = Field(default=5, ge=1, le=20, description="Number of chunks to retrieve")
    retrieval_similarity_threshold: float = Field(default=0.7, ge=0.0, le=1.0, description="Minimum similarity score")

    # Performance settings
    agent_timeout_seconds: int = Field(default=30, ge=5, le=60, description="Timeout for agent operations")
    max_query_length: int = Field(default=1000, ge=100, le=5000, description="Maximum query length")

    # Logging settings
    agent_log_level: str = Field(default="INFO", description="Log level for agent operations")


# Initialize agent settings
agent_settings = AgentSettings()

# Set up agent-specific logger
agent_logger = logging.getLogger("rag_agent")
agent_logger.setLevel(getattr(logging, agent_settings.agent_log_level))