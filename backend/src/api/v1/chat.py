"""
Chat API endpoints for the Full-Stack Integration feature.

This module contains FastAPI endpoints for the chat functionality
that connects the frontend to the backend RAG agent.
"""

import time
import logging
from typing import Optional
from uuid import uuid4
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from ...models.agent_models import UserQuery
from ...agents.rag_agent import RAGAgent
from ..v1.models.agent_models import AgentQueryRequest, AgentQueryResponse
from .models.chat_models import ChatRequest, ChatResponse, HealthResponse, ChatStatusResponse, ErrorApiResponse


router = APIRouter(prefix="/v1", tags=["chat"])
logger = logging.getLogger(__name__)

# Initialize the RAG agent
rag_agent = RAGAgent()


@router.options("/chat/query")
async def options_query():
    """Handle CORS preflight for chat query"""
    return {"message": "OK"}

@router.post("/chat/query", response_model=ChatResponse)
async def submit_query(request: ChatRequest):
    """
    Submit a query to the RAG agent and receive a grounded response with metadata.

    Args:
        request: The chat request containing the user's query

    Returns:
        ChatResponse with the agent's response and metadata
    """
    start_time = time.time()

    try:
        logger.info(f"Received query: {request.query[:50]}...")

        # Create an agent query request from the chat request
        agent_request = AgentQueryRequest(
            query=request.query,
            include_citations=request.include_citations,
            max_chunks=5,  # Default value, could be configurable
            similarity_threshold=0.7  # Default value, could be configurable
        )

        # Process the query with the RAG agent
        agent_response = await rag_agent.process_query(
            UserQuery(query_text=request.query)
        )

        # Calculate processing time
        processing_time_ms = int((time.time() - start_time) * 1000)

        # Generate a session_id if not provided in the request
        session_id = request.session_id or f"sess_{uuid4().hex}"

        # Create the chat response
        chat_response = ChatResponse(
            response=agent_response.content,
            citations=[],
            retrieved_chunks_count=len(agent_response.source_citations),
            processing_time_ms=processing_time_ms,
            confidence=agent_response.confidence_score,
            session_id=session_id
        )

        # Convert agent citations to chat citations
        for citation in agent_response.source_citations:
            chat_response.citations.append({
                "source_url": citation.get("source_url", ""),
                "document_id": citation.get("document_id", ""),
                "similarity_score": citation.get("similarity_score", 0.0),
                "content_preview": citation.get("content_preview", "")[:200] + "..." if len(citation.get("content_preview", "")) > 200 else citation.get("content_preview", "")
            })

        logger.info(f"Query processed successfully in {processing_time_ms}ms")
        return chat_response

    except Exception as e:
        logger.error(f"Error processing chat query: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")


@router.get("/chat/health", response_model=HealthResponse)
async def chat_health():
    """
    Check the health of the chat service.

    Returns:
        HealthResponse indicating the service status
    """
    try:
        # Perform basic health checks
        # For now, just return that the service is running
        health_response = HealthResponse(
            status="healthy",
            service="Chat Service",
            timestamp=str(time.time())
        )

        logger.info("Health check requested and responded successfully")
        return health_response

    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Health check failed: {str(e)}")


@router.get("/chat/status", response_model=ChatStatusResponse)
async def chat_status():
    """
    Get the status and statistics of the chat service.

    Returns:
        ChatStatusResponse with service statistics
    """
    try:
        # For now, return basic status information
        # In a real implementation, this would track actual metrics
        status_response = ChatStatusResponse(
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
def register_chat_routes(app):
    """
    Register the chat routes with the main application.

    Args:
        app: The FastAPI application instance
    """
    app.include_router(router, prefix="/v1", tags=["chat"])