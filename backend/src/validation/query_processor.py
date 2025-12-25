import time
from typing import List, Dict, Any, Optional
from ..services.embedding_service import EmbeddingService
from ..config.settings import settings
from ..config.validation_config import ValidationConfig
import logging


class QueryProcessor:
    """
    Query processing logic for the validation pipeline
    """

    def __init__(self, config: ValidationConfig):
        self.config = config
        self.logger = logging.getLogger(self.__class__.__name__)
        self.embedding_service = EmbeddingService(
            api_key=settings.cohere_api_key,
            model_name=settings.cohere_model
        )

    async def process_query(self, query_text: str) -> Dict[str, Any]:
        """
        Process a single query: generate embedding and validate format/dimensions
        """
        start_time = time.time()

        try:
            # Generate embedding for the query using Cohere
            query_embedding = await self.embedding_service.execute([query_text])
            query_embedding = query_embedding[0]  # Get the first (and only) embedding

            # Validate embedding properties
            embedding_validation = self.validate_embedding(
                query_embedding,
                expected_dimension=1024  # Cohere embeddings are typically 1024-dimensional
            )

            processing_time = time.time() - start_time

            return {
                "query_text": query_text,
                "query_embedding": query_embedding,
                "embedding_dimension": len(query_embedding),
                "embedding_validation": embedding_validation,
                "processing_time": processing_time,
                "timestamp": time.time()
            }

        except Exception as e:
            self.logger.error(f"Error processing query '{query_text}': {str(e)}")
            return {
                "query_text": query_text,
                "error": str(e),
                "embedding_validation": {
                    "is_valid": False,
                    "error": str(e)
                },
                "processing_time": time.time() - start_time,
                "timestamp": time.time()
            }

    async def process_batch_queries(self, queries: List[str]) -> List[Dict[str, Any]]:
        """
        Process multiple queries in batch
        """
        results = []
        for query in queries:
            result = await self.process_query(query)
            results.append(result)
        return results

    def validate_embedding(self, embedding: List[float], expected_dimension: int = 1024) -> Dict[str, Any]:
        """
        Validate that the embedding has the correct format and dimensions
        """
        validation_result = {
            "is_valid": True,
            "dimension_valid": True,
            "format_valid": True,
            "errors": []
        }

        # Check if embedding is a list of floats
        if not isinstance(embedding, list):
            validation_result["is_valid"] = False
            validation_result["format_valid"] = False
            validation_result["errors"].append("Embedding is not a list")
        else:
            # Check each element is a float
            for i, val in enumerate(embedding):
                if not isinstance(val, (int, float)):
                    validation_result["is_valid"] = False
                    validation_result["format_valid"] = False
                    validation_result["errors"].append(f"Element at index {i} is not a number: {type(val)}")
                    break

        # Check dimension
        if len(embedding) != expected_dimension:
            validation_result["is_valid"] = False
            validation_result["dimension_valid"] = False
            validation_result["errors"].append(f"Dimension mismatch: expected {expected_dimension}, got {len(embedding)}")

        return validation_result

    def compare_embeddings(self, query_embedding: List[float], stored_embedding: List[float]) -> Dict[str, Any]:
        """
        Compare query embedding with stored embeddings for consistency
        """
        if len(query_embedding) != len(stored_embedding):
            return {
                "consistent": False,
                "similarity": 0.0,
                "error": "Dimension mismatch"
            }

        # Calculate cosine similarity
        similarity = self._cosine_similarity(query_embedding, stored_embedding)

        # Determine consistency based on threshold
        consistent = similarity >= self.config.validation_threshold

        return {
            "consistent": consistent,
            "similarity": similarity,
            "consistent_with_threshold": consistent
        }

    def _cosine_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """
        Calculate cosine similarity between two vectors
        """
        if len(vec1) != len(vec2):
            raise ValueError("Vectors must have the same length")

        # Calculate dot product
        dot_product = sum(a * b for a, b in zip(vec1, vec2))

        # Calculate magnitudes
        magnitude1 = sum(a * a for a in vec1) ** 0.5
        magnitude2 = sum(a * a for a in vec2) ** 0.5

        if magnitude1 == 0 or magnitude2 == 0:
            return 0.0

        return dot_product / (magnitude1 * magnitude2)

    def validate_semantic_consistency(self, query_embeddings: List[List[float]], stored_embeddings: List[List[float]]) -> Dict[str, Any]:
        """
        Validate semantic consistency between query and stored embeddings
        """
        if not query_embeddings or not stored_embeddings:
            return {
                "consistent": False,
                "average_similarity": 0.0,
                "validation_details": "No embeddings to compare"
            }

        similarities = []
        for query_emb in query_embeddings:
            for stored_emb in stored_embeddings:
                similarity = self._cosine_similarity(query_emb, stored_emb)
                similarities.append(similarity)

        if not similarities:
            return {
                "consistent": False,
                "average_similarity": 0.0,
                "validation_details": "No similarity comparisons made"
            }

        average_similarity = sum(similarities) / len(similarities)
        consistent = average_similarity >= self.config.validation_threshold

        return {
            "consistent": consistent,
            "average_similarity": average_similarity,
            "min_similarity": min(similarities),
            "max_similarity": max(similarities),
            "total_comparisons": len(similarities)
        }

    async def execute(self, queries: List[str]) -> List[Dict[str, Any]]:
        """
        Execute the query processing operation
        """
        return await self.process_batch_queries(queries)