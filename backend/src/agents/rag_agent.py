"""
RAG Agent implementation for the RAG Agent Construction feature.

This module contains the main RAG agent class that can answer user queries
by dynamically invoking retrieval over the validated RAG pipeline and
synthesizing grounded responses.
"""

import logging
from typing import Dict, List, Optional, Any
from openai import OpenAI
from ..models.agent_models import UserQuery, RetrievedChunk, AgentResponse
from ..services.retrieval_service import RetrievalService
from ..config.agent_config import agent_settings
from ..config.settings import settings
from .tools.retrieval_tool import RetrievalTool
from .tools.tool_registry import tool_registry


class RAGAgent:
    """Main RAG agent class that processes user queries using retrieval-augmented generation"""

    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        # Initialize OpenAI client with OpenRouter if configured
        try:
            if settings.openrouter_api_key:
                self.openai_client = OpenAI(
                    base_url=settings.openrouter_base_url,
                    api_key=settings.openrouter_api_key,
                )
                self.model = settings.openrouter_model
            else:
                self.openai_client = OpenAI()
                self.model = agent_settings.agent_model
        except Exception as e:
            self.logger.error(f"Error initializing OpenAI client: {str(e)}")
            # Fallback to a dummy client or raise an exception to handle gracefully
            raise e

        self.retrieval_service = RetrievalService()
        self.tool_registry = tool_registry
        
        # Other settings
        self.temperature = agent_settings.agent_temperature
        self.max_tokens = agent_settings.agent_max_tokens

    async def process_query(self, user_query: UserQuery) -> AgentResponse:
        """
        Process a user query and return a grounded response

        Args:
            user_query: The user query to process

        Returns:
            AgentResponse with the answer and citations
        """
        self.logger.info(f"Processing query: {user_query.query_text[:50]}...")

        try:
            # Determine if retrieval is needed for this query
            should_retrieve = await self._should_invoke_retrieval(user_query.query_text)

            if should_retrieve:
                # Retrieve relevant chunks
                retrieved_chunks = await self.retrieval_service.retrieve_chunks(
                    user_query.query_text,
                    top_k=agent_settings.retrieval_top_k,
                    similarity_threshold=agent_settings.retrieval_similarity_threshold
                )

                # Generate response using retrieved content
                response_content = await self._generate_response_with_retrieval(
                    user_query.query_text, retrieved_chunks
                )

                # Create citations from retrieved chunks
                citations = self._create_citations(retrieved_chunks)

                # Calculate confidence based on retrieval quality
                confidence = self._calculate_confidence(retrieved_chunks)
            else:
                # Generate response without retrieval
                response_content = await self._generate_response_without_retrieval(user_query.query_text)
                citations = []
                confidence = 0.5  # Default confidence for non-retrieval responses

            # Create and return the response
            response = AgentResponse(
                content=response_content,
                source_citations=citations,
                confidence_score=confidence,
                query_id=user_query.query_id
            )

            self.logger.info(f"Query processed successfully, confidence: {confidence}")
            return response

        except Exception as e:
            self.logger.error(f"Error processing query: {str(e)}")
            # Return a response indicating the error
            return AgentResponse(
                content="I encountered an error while processing your query. Please try again later.",
                source_citations=[],
                confidence_score=0.0,
                query_id=user_query.query_id
            )

    async def _should_invoke_retrieval(self, query_text: str) -> bool:
        """
        Determine if retrieval should be invoked for the given query
        """
        # Simple classification: Conversational/greetings or general off-topic
        conversational_keywords = ["hello", "hi", "how are you", "who are you", "what can you do", "thank", "joke", "weather"]
        query_lower = query_text.lower()
        
        # Check if it's a short conversational query
        if any(keyword in query_lower for keyword in conversational_keywords) and len(query_lower.split()) < 12:
            self.logger.info("Query classified as conversational, skipping retrieval")
            return False
            
        return True

    async def _generate_response_with_retrieval(self, query: str, retrieved_chunks: List[RetrievedChunk]) -> str:
        """
        Generate a response using the retrieved chunks as context

        Args:
            query: The original user query
            retrieved_chunks: List of retrieved chunks to use as context

        Returns:
            Generated response string
        """
        # Create context from retrieved chunks
        context = self._format_retrieved_chunks(retrieved_chunks)

        # Create a strict system prompt with guardrails
        prompt = f"""
        You are the "Physical AI & Humanoid Robotics Textbook" assistant.
        
        STRICT GUARDRAILS:
        1. Only answer questions related to Physical AI, Robotics, ROS2, Computer Vision, or Humanoid systems.
        2. Use ONLY the provided context to answer technical questions.
        3. If the context does not contain the answer TO A RELEVANT TECHNICAL QUESTION, say "I'm sorry, but I don't have enough information in the textbook to answer that specifically."
        4. CRITICAL: If the user asks about anything unrelated (politics, food, sports, general knowledge, etc.), YOU MUST REFUSE by saying: "I am specialized ONLY in Physical AI and Humanoid Robotics. I cannot answer questions outside of this domain."
        5. Do not hallucinate or provide information from external knowledge for technical topics.
        6. Tone: Academic, helpful, and concise.

        Context:
        {context}

        Question: {query}

        Answer:
        """

        try:
            response = self.openai_client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1,  # Lower temperature for more grounded response
                max_tokens=self.max_tokens
            )

            return response.choices[0].message.content.strip()
        except Exception as e:
            self.logger.error(f"Error generating response with OpenAI: {str(e)}")
            return f"I encountered an error while generating a response: {str(e)}"

    async def _generate_response_without_retrieval(self, query: str) -> str:
        """
        Generate a response without using retrieval (for greetings or out-of-scope refusals)
        """
        prompt = f"""
        You are the "Physical AI & Humanoid Robotics Textbook" assistant.
        
        STRICT GUARDRAILS:
        1. If the user query is a greeting (Hi, Hello), respond politely and mention you are here to help with Physical AI and Robotics.
        2. If the user query is NOT about Physical AI, Robotics, or related technical fields, politely refuse: "I am specialized only in Physical AI and Humanoid Robotics. I cannot answer questions outside of this domain."
        3. Do not engage in general conversation unrelated to the textbook's subject matter.

        Query: {query}

        Answer:
        """

        try:
            response = self.openai_client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=self.temperature,
                max_tokens=self.max_tokens
            )

            return response.choices[0].message.content.strip()
        except Exception as e:
            self.logger.error(f"Error generating response without retrieval: {str(e)}")
            return f"I encountered an error while generating a response: {str(e)}"

    def _format_retrieved_chunks(self, chunks: List[RetrievedChunk]) -> str:
        """
        Format retrieved chunks into a context string

        Args:
            chunks: List of retrieved chunks

        Returns:
            Formatted context string
        """
        formatted_chunks = []
        for i, chunk in enumerate(chunks, 1):
            formatted_chunk = f"""
            Document {i}:
            Content: {chunk.content}
            Source: {chunk.source_url}
            Document ID: {chunk.document_id}
            Similarity Score: {chunk.similarity_score}
            """
            formatted_chunks.append(formatted_chunk)

        return "\n".join(formatted_chunks)

    def _create_citations(self, chunks: List[RetrievedChunk]) -> List[Dict[str, Any]]:
        """
        Create citations from retrieved chunks

        Args:
            chunks: List of retrieved chunks

        Returns:
            List of citation dictionaries
        """
        citations = []
        for chunk in chunks:
            citation = {
                "source_url": chunk.source_url,
                "document_id": chunk.document_id,
                "similarity_score": chunk.similarity_score,
                "content_preview": chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content
            }
            citations.append(citation)

        return citations

    def _calculate_confidence(self, chunks: List[RetrievedChunk]) -> float:
        """
        Calculate confidence score based on retrieved chunks

        Args:
            chunks: List of retrieved chunks

        Returns:
            Confidence score between 0.0 and 1.0
        """
        if not chunks:
            return 0.0

        # Calculate average similarity score
        avg_similarity = sum(chunk.similarity_score for chunk in chunks) / len(chunks)

        # Weight the confidence based on number of chunks and their quality
        num_chunks_factor = min(len(chunks) / 5.0, 1.0)  # Cap at 1.0 for 5+ chunks

        # Combine factors to calculate overall confidence
        confidence = (avg_similarity * 0.7 + num_chunks_factor * 0.3)
        return min(confidence, 1.0)  # Ensure it doesn't exceed 1.0