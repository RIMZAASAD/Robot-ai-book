"""
Retrieval service wrapper for the RAG Agent Construction feature.

This module provides a wrapper around the existing retrieval pipeline
to be used by the agent as a tool.
"""

import logging
from typing import List, Optional
from ..models.agent_models import RetrievedChunk
from ..config.agent_config import agent_settings


class RetrievalService:
    """Wrapper for the retrieval functionality to be used by the agent"""

    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        # Import the existing vector search service from the validation pipeline
        try:
            from ..services.vector_search import VectorSearchService
            from ..services.embedding_service import EmbeddingService
            self.vector_search_service = VectorSearchService()
            self.embedding_service = EmbeddingService()
        except ImportError:
            self.logger.warning("Services not found, using mock implementation")
            self.vector_search_service = None
            self.embedding_service = None

    async def retrieve_chunks(self, query_text: str, top_k: Optional[int] = None,
                            similarity_threshold: Optional[float] = None) -> List[RetrievedChunk]:
        """
        Retrieve relevant document chunks based on the query

        Args:
            query_text: The query text to search for
            top_k: Number of chunks to retrieve (defaults to config value)
            similarity_threshold: Minimum similarity score (defaults to config value)

        Returns:
            List of RetrievedChunk objects
        """
        if top_k is None:
            top_k = agent_settings.retrieval_top_k
        if similarity_threshold is None:
            similarity_threshold = agent_settings.retrieval_similarity_threshold

        self.logger.info(f"Retrieving chunks for query: {query_text[:50]}...")

        try:
            if self.vector_search_service:
                # Use the existing vector search service
                search_results = self.vector_search_service.search_similar_chunks(
                    query_embedding=await self._get_query_embedding(query_text),
                    top_k=top_k,
                    threshold=similarity_threshold
                )

                # Convert search results to RetrievedChunk objects
                retrieved_chunks = []
                for result in search_results:
                    chunk = RetrievedChunk(
                        chunk_id=result.get('id', ''),
                        content=result.get('content', ''),
                        similarity_score=result.get('score', 0.0),
                        source_url=result.get('source_url', ''),
                        document_id=result.get('document_id', ''),
                        chunk_index=result.get('metadata', {}).get('chunk_index', 0),
                        metadata=result.get('metadata', {})
                    )
                    retrieved_chunks.append(chunk)

                self.logger.info(f"Retrieved {len(retrieved_chunks)} chunks")
                return retrieved_chunks
            else:
                # Mock implementation for testing
                self.logger.warning("Using mock retrieval service")
                return self._mock_retrieve_chunks(query_text, top_k)
        except Exception as e:
            self.logger.error(f"Error in retrieval: {str(e)}")
            return []

    async def _get_query_embedding(self, query_text: str) -> List[float]:
        """Get embedding for query text"""
        if self.embedding_service:
            embeddings = await self.embedding_service.execute([query_text], input_type="search_query")
            return embeddings[0]
        return [0.0] * 1024  # Default empty vector

    def _mock_retrieve_chunks(self, query_text: str, top_k: int) -> List[RetrievedChunk]:
        """Mock implementation for testing purposes"""
        # This is just a placeholder - in real implementation, this would connect
        # to the actual retrieval pipeline
        mock_chunks = []
        for i in range(min(top_k, 3)):  # Return max 3 mock chunks
            chunk = RetrievedChunk(
                chunk_id=f"mock_chunk_{i}",
                content=f"This is a mock retrieved chunk related to: {query_text}",
                similarity_score=0.8 + (i * 0.05),  # Slightly decreasing similarity
                source_url="https://example.com/mock-source",
                document_id=f"mock_doc_{i}",
                chunk_index=i,
                metadata={"mock": True}
            )
            mock_chunks.append(chunk)

        return mock_chunks

    async def validate_retrieval(self, query_text: str, expected_keywords: List[str]) -> bool:
        """
        Validate that retrieval returns relevant content for the query

        Args:
            query_text: The query to validate
            expected_keywords: Keywords that should appear in retrieved content

        Returns:
            True if validation passes, False otherwise
        """
        chunks = await self.retrieve_chunks(query_text, top_k=3)

        # Check if any of the expected keywords appear in the retrieved content
        content_text = " ".join([chunk.content.lower() for chunk in chunks])
        keyword_matches = [kw.lower() for kw in expected_keywords if kw.lower() in content_text]

        self.logger.info(f"Validation: Found {len(keyword_matches)} out of {len(expected_keywords)} expected keywords")
        return len(keyword_matches) >= len(expected_keywords) * 0.5  # At least 50% match