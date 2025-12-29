"""
API endpoints for the RAG Agent Construction feature.

This module contains FastAPI endpoints for interacting with the RAG agent.
"""

import time
import logging
from typing import Optional
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from ...models.agent_models import UserQuery, AgentResponse
from ...agents.rag_agent import RAGAgent
from ..v1.models.agent_models import AgentQueryRequest, AgentQueryResponse, HealthResponse, AgentStatusResponse


router = APIRouter(prefix="/v1", tags=["agent"])
logger = logging.getLogger(__name__)

# Initialize the RAG agent
rag_agent = RAGAgent()


@router.post("/agent/query", response_model=AgentQueryResponse)
async def query_agent(request: AgentQueryRequest):
    """
    Submit a query to the RAG agent and receive a grounded response.

    Args:
        request: The query request containing the user's question

    Returns:
        AgentQueryResponse with the agent's response and metadata
    """
    start_time = time.time()

    try:
        logger.info(f"Received query: {request.query[:50]}...")

        # Create a UserQuery object from the request
        user_query = UserQuery(
            query_text=request.query,
            metadata={
                "include_citations": request.include_citations,
                "max_chunks": request.max_chunks,
                "similarity_threshold": request.similarity_threshold
            }
        )

        # Process the query with the RAG agent
        agent_response = await rag_agent.process_query(user_query)

        # Calculate processing time
        processing_time_ms = int((time.time() - start_time) * 1000)

        # Count retrieved chunks from citations
        retrieved_chunks_count = len(agent_response.source_citations)

        # Create the API response
        api_response = AgentQueryResponse(
            response_id=agent_response.response_id,
            response=agent_response.content,
            citations=agent_response.source_citations,
            retrieved_chunks_count=retrieved_chunks_count,
            processing_time_ms=processing_time_ms,
            confidence=agent_response.confidence_score
        )

        logger.info(f"Query processed successfully in {processing_time_ms}ms")
        return api_response

    except Exception as e:
        logger.error(f"Error processing agent query: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")


@router.get("/agent/health", response_model=HealthResponse)
async def agent_health():
    """
    Check the health of the agent service.

    Returns:
        HealthResponse indicating the service status
    """
    try:
        # Perform basic health checks
        # For now, just return that the service is running
        health_response = HealthResponse(
            status="healthy",
            service="RAG Agent Service",
            timestamp=str(time.time())
        )

        logger.info("Health check requested and responded successfully")
        return health_response

    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Health check failed: {str(e)}")


@router.get("/agent/status", response_model=AgentStatusResponse)
async def agent_status():
    """
    Get the status and statistics of the agent service.

    Returns:
        AgentStatusResponse with service statistics
    """
    try:
        # For now, return basic status information
        # In a real implementation, this would track actual metrics
        status_response = AgentStatusResponse(
            status="running",
            total_queries=0,  # This would come from a metrics store in a real implementation
            successful_queries=0,
            success_rate=100.0,
            avg_response_time_ms=0,
            last_updated=str(time.time())
        )

        logger.info("Status check requested and responded successfully")
        return status_response

    except Exception as e:
        logger.error(f"Status check failed: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Status check failed: {str(e)}")


# Update main.py to include these routes
def register_agent_routes(app):
    """
    Register the agent routes with the main application.

    Args:
        app: The FastAPI application instance
    """
    app.include_router(router, prefix="/v1", tags=["agent"])