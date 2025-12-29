"""
Retrieval tool for the RAG Agent Construction feature.

This module implements a custom tool that allows the agent to retrieve
relevant document chunks from the RAG pipeline.
"""

import logging
from typing import Any, Dict, List
from pydantic import BaseModel, Field
from ...services.retrieval_service import RetrievalService


class RetrievalToolInput(BaseModel):
    """Input schema for the retrieval tool"""

    query: str = Field(..., description="The query to search for in the document collection")
    top_k: int = Field(default=5, ge=1, le=20, description="Number of chunks to retrieve")
    similarity_threshold: float = Field(default=0.7, ge=0.0, le=1.0, description="Minimum similarity score")


class RetrievalTool:
    """Custom tool for the agent to retrieve relevant document chunks"""

    def __init__(self):
        self.name = "retrieval_tool"
        self.description = "Retrieve relevant document chunks from the RAG pipeline based on a query"
        self.input_schema = RetrievalToolInput
        self.logger = logging.getLogger(self.__class__.__name__)
        self.retrieval_service = RetrievalService()

    async def run(self, input_data: RetrievalToolInput) -> Dict[str, Any]:
        """
        Execute the retrieval tool with the given input

        Args:
            input_data: Input data containing the query and parameters

        Returns:
            Dictionary containing the retrieval results
        """
        self.logger.info(f"Running retrieval tool for query: {input_data.query[:50]}...")

        try:
            # Retrieve chunks using the retrieval service
            retrieved_chunks = await self.retrieval_service.retrieve_chunks(
                query_text=input_data.query,
                top_k=input_data.top_k,
                similarity_threshold=input_data.similarity_threshold
            )

            # Format the results
            results = {
                "query": input_data.query,
                "retrieved_chunks": [
                    {
                        "content": chunk.content,
                        "source_url": chunk.source_url,
                        "document_id": chunk.document_id,
                        "similarity_score": chunk.similarity_score,
                        "chunk_index": chunk.chunk_index
                    }
                    for chunk in retrieved_chunks
                ],
                "total_chunks": len(retrieved_chunks)
            }

            self.logger.info(f"Retrieved {len(retrieved_chunks)} chunks for query")
            return results

        except Exception as e:
            self.logger.error(f"Error in retrieval tool: {str(e)}")
            return {
                "error": str(e),
                "query": input_data.query,
                "retrieved_chunks": [],
                "total_chunks": 0
            }

    def to_dict(self) -> Dict[str, Any]:
        """Convert the tool to a dictionary representation for OpenAI agent"""
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": self.input_schema.model_json_schema()
            }
        }