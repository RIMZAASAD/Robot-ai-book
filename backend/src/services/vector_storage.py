from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
import uuid
from .base_service import BaseService
from ..config.qdrant_config import QdrantConfig


class VectorStorageService(BaseService):
    """
    Service for storing embeddings in Qdrant vector database
    """

    def __init__(self, collection_name: str = None):
        super().__init__()
        self.qdrant_config = QdrantConfig()
        self.client = self.qdrant_config.get_client()
        self.collection_name = collection_name or self.qdrant_config.collection_name

    def store_embeddings(self, embeddings: List[List[float]],
                        texts: List[str],
                        metadata_list: List[Dict[str, Any]]) -> List[str]:
        """
        Store embeddings in Qdrant with metadata
        """
        if len(embeddings) != len(texts) or len(embeddings) != len(metadata_list):
            raise ValueError("Embeddings, texts, and metadata lists must have the same length")

        # Prepare points for insertion
        points = []
        ids = []

        for i, (embedding, text, metadata) in enumerate(zip(embeddings, texts, metadata_list)):
            point_id = str(uuid.uuid4())
            ids.append(point_id)

            point = models.PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "text": text,
                    "title": metadata.get("title", ""),
                    "url": metadata.get("url", ""),
                    "chunk_index": metadata.get("chunk_index", 0),
                    "document_id": metadata.get("document_id", ""),
                    "created_at": metadata.get("created_at", ""),
                    "source_type": metadata.get("source_type", "web_page")
                }
            )
            points.append(point)

        # Upload points to Qdrant
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

        self.logger.info(f"Stored {len(points)} embeddings in Qdrant collection '{self.collection_name}'")
        return ids

    def search_similar(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in Qdrant
        """
        try:
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit
            )

            results = []
            for result in search_results:
                results.append({
                    "id": result.id,
                    "text": result.payload.get("text", ""),
                    "url": result.payload.get("url", ""),
                    "title": result.payload.get("title", ""),
                    "score": result.score,
                    "metadata": result.payload
                })

            return results
        except Exception as e:
            error_result = self.handle_error(e, "Vector search")
            return []

    def get_vector_count(self) -> int:
        """
        Get the total count of vectors in the collection
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return collection_info.points_count
        except Exception as e:
            error_result = self.handle_error(e, "Get vector count")
            return 0

    def delete_by_payload(self, key: str, value: str) -> bool:
        """
        Delete vectors by payload filter
        """
        try:
            # Create filter to find points with specific payload
            filter_condition = models.Filter(
                must=[
                    models.FieldCondition(
                        key=f"payload.{key}",
                        match=models.MatchValue(value=value)
                    )
                ]
            )

            # Delete points matching the filter
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.FilterSelector(filter=filter_condition)
            )

            return True
        except Exception as e:
            error_result = self.handle_error(e, f"Delete by payload: {key}={value}")
            return False

    async def execute(self, embeddings: List[List[float]], texts: List[str], metadata_list: List[Dict[str, Any]]) -> List[str]:
        """
        Execute the vector storage operation
        """
        return self.store_embeddings(embeddings, texts, metadata_list)