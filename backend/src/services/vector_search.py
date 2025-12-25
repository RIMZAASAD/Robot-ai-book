from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from ..config.settings import settings
import logging


class VectorSearchService:
    """
    Service for searching the Qdrant vector database for semantically similar chunks
    """

    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            host=settings.qdrant_host,
            port=settings.qdrant_port
        )
        self.collection_name = settings.collection_name

    def search_similar_chunks(self, query_embedding: List[float], top_k: int = 5, threshold: float = 0.0) -> List[Dict[str, Any]]:
        """
        Search for similar chunks in Qdrant based on cosine similarity
        """
        try:
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                score_threshold=threshold
            )

            results = []
            for i, result in enumerate(search_results):
                results.append({
                    "id": result.id,
                    "content": result.payload.get("text", ""),
                    "source_url": result.payload.get("url", ""),
                    "document_id": result.payload.get("document_id", ""),
                    "score": result.score,
                    "metadata": result.payload,
                    "rank": i + 1
                })

            self.logger.info(f"Found {len(results)} similar chunks for query")
            return results
        except Exception as e:
            self.logger.error(f"Error searching for similar chunks: {str(e)}")
            raise

    def get_chunk_by_id(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific chunk by its ID
        """
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[chunk_id]
            )

            if records:
                record = records[0]
                return {
                    "id": record.id,
                    "content": record.payload.get("text", ""),
                    "source_url": record.payload.get("url", ""),
                    "document_id": record.payload.get("document_id", ""),
                    "metadata": record.payload
                }
            return None
        except Exception as e:
            self.logger.error(f"Error retrieving chunk {chunk_id}: {str(e)}")
            return None

    def batch_search(self, query_embeddings: List[List[float]], top_k: int = 5) -> List[List[Dict[str, Any]]]:
        """
        Perform batch search for multiple query embeddings
        """
        results = []
        for embedding in query_embeddings:
            result = self.search_similar_chunks(embedding, top_k)
            results.append(result)
        return results

    def validate_search_results(self, query: str, results: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Validate search results for relevance and metadata completeness
        """
        validation_result = {
            "query": query,
            "total_results": len(results),
            "results_with_metadata": 0,
            "results_with_content": 0,
            "avg_similarity_score": 0.0,
            "metadata_completeness": 0.0,
            "content_validity": 0.0
        }

        if results:
            total_similarity = 0.0
            for result in results:
                if result.get("source_url") and result.get("document_id"):
                    validation_result["results_with_metadata"] += 1
                if result.get("content"):
                    validation_result["results_with_content"] += 1
                total_similarity += result.get("score", 0.0)

            validation_result["avg_similarity_score"] = total_similarity / len(results)
            validation_result["metadata_completeness"] = validation_result["results_with_metadata"] / len(results)
            validation_result["content_validity"] = validation_result["results_with_content"] / len(results)

        return validation_result