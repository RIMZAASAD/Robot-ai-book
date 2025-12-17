from fastapi import APIRouter, HTTPException
import time
import logging
from typing import List, Dict, Any
from ...models.chat import ChatRequest, SelectedTextChatRequest, ChatResponse, SelectedTextChatResponse
from ...services.openrouter_service import openrouter_service
from ...services.embedding_service import embedding_service
from ...services.qdrant_service import qdrant_service
from ...services.content_ingestion import content_ingestion_service
from ...services.postgres_service import postgres_service

logger = logging.getLogger(__name__)
router = APIRouter()

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """
    Chat endpoint that uses RAG to answer questions from textbook content
    """
    start_time = time.time()

    try:
        # Generate a session ID if not provided
        session_id = request.session_id or f"session_{int(time.time())}"

        # Store session in PostgreSQL if it's a new session
        if not request.session_id:
            session_data = {
                "session_id": session_id,
                "user_id": None,  # Could be populated if we have user authentication
                "page_history": [request.page_context] if request.page_context else [],
                "mode_preference": "full_textbook"
            }
            await postgres_service.store_session(session_data)

        # Generate embedding for the question
        question_embedding = await embedding_service.generate_embedding(request.question)

        # Search for relevant content in Qdrant
        search_results = await qdrant_service.search_similar(
            query_embedding=question_embedding,
            limit=5,
            filters={"chapter": request.page_context} if request.page_context else None
        )

        if not search_results:
            # If no relevant content found, return a helpful response
            response_text = "I couldn't find relevant information in the textbook to answer your question. Please try rephrasing or consult the relevant chapter."
            confidence = 0.0
            citations = []
        else:
            # Build context from search results
            context_parts = []
            citations = []
            for result in search_results:
                context_parts.append(result["text"])
                citations.append({
                    "content_id": result["id"],
                    "title": result["title"],
                    "chapter": result["chapter"],
                    "text_preview": result["text"][:200] + "..." if len(result["text"]) > 200 else result["text"]
                })

            context = "\n\n".join(context_parts)

            # Create messages for the LLM
            messages = [
                {
                    "role": "system",
                    "content": "You are an assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based only on the provided context from the textbook. If the answer is not in the context, say so clearly."
                },
                {
                    "role": "user",
                    "content": f"Context: {context}\n\nQuestion: {request.question}\n\nPlease provide an answer based on the context provided. If the answer is not in the context, please state this clearly."
                }
            ]

            # Generate response using OpenRouter
            response_text = await openrouter_service.generate_response(messages)

            # Calculate confidence based on search scores
            avg_score = sum(r["score"] for r in search_results) / len(search_results)
            confidence = min(avg_score, 1.0)  # Cap at 1.0

        response_time = time.time() - start_time

        # Store chat history in PostgreSQL
        chat_data = {
            "session_id": session_id,
            "question": request.question,
            "response": response_text,
            "confidence_score": confidence,
            "citations": citations
        }
        await postgres_service.store_chat_history(chat_data)

        return ChatResponse(
            response=response_text,
            confidence_score=confidence,
            citations=citations,
            session_id=session_id,
            response_time=response_time
        )

    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.post("/chat-selected", response_model=SelectedTextChatResponse)
async def chat_selected_endpoint(request: SelectedTextChatRequest):
    """
    Chat endpoint that answers questions based only on selected text
    """
    start_time = time.time()

    try:
        # Generate a session ID if not provided
        session_id = request.session_id or f"session_{int(time.time())}"

        # Store session in PostgreSQL if it's a new session
        if not request.session_id:
            session_data = {
                "session_id": session_id,
                "user_id": None,  # Could be populated if we have user authentication
                "page_history": [],  # Could be populated with current page
                "mode_preference": "selected_text_only"
            }
            await postgres_service.store_session(session_data)

        # Generate embedding for the question
        question_embedding = await embedding_service.generate_embedding(request.question)

        # Search for content related to the selected text
        search_results = await qdrant_service.search_by_selected_text(
            selected_text=request.selected_text,
            query_embedding=question_embedding,
            limit=3
        )

        # For selected text mode, we want to ensure the response is based ONLY on the selected text
        # Build context using the selected text and any closely related content
        context = f"USER SELECTED TEXT: {request.selected_text}"

        # Add related content if found
        for result in search_results:
            if result["text"] != request.selected_text and len(context) < 3000:  # Prevent context from getting too large
                context += f"\n\nRELATED CONTEXT: {result['text']}"

        # Create messages for the LLM
        messages = [
            {
                "role": "system",
                "content": "You are an assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based ONLY on the provided selected text. Do not use any external knowledge or information not present in the selected text. Be strict about this requirement."
            },
            {
                "role": "user",
                "content": f"SELECTED TEXT: {context}\n\nQuestion: {request.question}\n\nAnswer ONLY based on the selected text provided above. Do not use any external knowledge or information not present in the selected text."
            }
        ]

        # Generate response using OpenRouter
        response_text = await openrouter_service.generate_response(messages)

        # Calculate confidence
        confidence = 0.8 if search_results else 0.3  # Default confidence values

        response_time = time.time() - start_time

        # Store chat history in PostgreSQL
        chat_data = {
            "session_id": session_id,
            "question": request.question,
            "response": response_text,
            "confidence_score": confidence,
            "citations": {"selected_text": request.selected_text[:200] + "..." if len(request.selected_text) > 200 else request.selected_text}
        }
        await postgres_service.store_chat_history(chat_data)

        return SelectedTextChatResponse(
            response=response_text,
            confidence_score=confidence,
            source_text_used=request.selected_text,
            session_id=session_id,
            response_time=response_time
        )

    except Exception as e:
        logger.error(f"Error in chat-selected endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")