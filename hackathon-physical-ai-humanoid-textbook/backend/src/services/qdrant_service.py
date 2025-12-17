import os
import logging
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct
import uuid

logger = logging.getLogger(__name__)

class QdrantService:
    def __init__(self):
        # Get Qdrant configuration from environment
        self.url = os.getenv("QDRANT_URL")
        self.api_key = os.getenv("QDRANT_API_KEY")

        if not self.url:
            logger.warning("QDRANT_URL environment variable not set, service will not be available")
            self.is_available = False
            self.client = None
            return
        else:
            self.is_available = True

        # Initialize Qdrant client
        # Handle both cloud and local instances properly
        if "https://" in self.url or "http://" in self.url:
            # Cloud instance - use full URL
            self.client = QdrantClient(
                url=self.url,
                api_key=self.api_key,
                timeout=10.0
            )
        elif self.api_key:
            # Local instance with API key
            self.client = QdrantClient(
                url=self.url,
                api_key=self.api_key,
                timeout=10.0
            )
        else:
            # Local instance without API key
            self.client = QdrantClient(host="localhost", port=6333)

        # Collection name for textbook content
        self.collection_name = "textbook_content"

        # Initialize the collection
        self._initialize_collection()

    def _initialize_collection(self):
        """Initialize the Qdrant collection if it doesn't exist"""
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                # Create collection with appropriate vector size for our embedding model (384 for all-MiniLM-L6-v2)
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=384, distance=Distance.COSINE)  # Changed from 1536 to 384
                )
                logger.info(f"Created Qdrant collection: {self.collection_name} with 384 dimensions")
            else:
                # Get the existing collection info to check dimensions
                collection_info = self.client.get_collection(self.collection_name)
                vector_size = collection_info.config.params.vectors.size

                if vector_size != 384:
                    logger.warning(f"Existing collection has {vector_size} dimensions, but we need 384. "
                                  f"Please recreate the collection or re-ingest data with correct dimensions.")
                else:
                    logger.info(f"Qdrant collection {self.collection_name} already exists with correct dimensions (384)")

        except Exception as e:
            logger.error(f"Error initializing Qdrant collection: {str(e)}")
            raise

    async def store_embeddings(self,
                             texts: List[str],
                             embeddings: List[List[float]],
                             metadata_list: List[Dict[str, Any]]) -> List[str]:
        """
        Store embeddings in Qdrant with metadata
        """
        if not self.is_available:
            logger.error("Qdrant service not available. Cannot store embeddings.")
            return []  # Return empty list when service is not available

        if len(texts) != len(embeddings) or len(embeddings) != len(metadata_list):
            raise ValueError("Texts, embeddings, and metadata lists must have the same length")

        # Prepare points for insertion
        points = []
        ids = []

        for i, (text, embedding, metadata) in enumerate(zip(texts, embeddings, metadata_list)):
            point_id = str(uuid.uuid4())
            ids.append(point_id)

            point = PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "text": text,
                    "title": metadata.get("title", ""),
                    "chapter": metadata.get("chapter", ""),
                    "section": metadata.get("section", ""),
                    "source_file": metadata.get("source_file", ""),
                    "created_at": metadata.get("created_at", "")
                }
            )
            points.append(point)

        # Upload points to Qdrant
        self.client.upsert(
            collection_name=self.collection_name,
            points=points
        )

        logger.info(f"Stored {len(points)} embeddings in Qdrant")
        return ids

    async def search_similar(self,
                           query_embedding: List[float],
                           limit: int = 5,
                           filters: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """
        Search for similar content based on embedding
        """
        if not self.is_available:
            logger.error("Qdrant service not available. Cannot perform search.")
            return []  # Return empty list when service is not available

        try:
            # Prepare filters if provided
            qdrant_filters = None
            if filters:
                filter_conditions = []
                for key, value in filters.items():
                    filter_conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        )
                    )

                if filter_conditions:
                    qdrant_filters = models.Filter(
                        must=filter_conditions
                    )

            # Search in Qdrant using query_points method
            search_response = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=limit,
                query_filter=qdrant_filters,
                with_payload=True,
                with_vectors=False
            )

            # Format results from the response object
            results = []
            for hit in search_response.points:  # Access the points attribute
                result = {
                    "id": hit.id,
                    "text": hit.payload.get("text", ""),
                    "title": hit.payload.get("title", ""),
                    "chapter": hit.payload.get("chapter", ""),
                    "section": hit.payload.get("section", ""),
                    "source_file": hit.payload.get("source_file", ""),
                    "score": hit.score
                }
                results.append(result)

            return results

        except Exception as e:
            logger.error(f"Error searching in Qdrant: {str(e)}")
            raise

    async def search_by_selected_text(self,
                                    selected_text: str,
                                    query_embedding: List[float],
                                    limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for content that contains the selected text
        """
        if not self.is_available:
            logger.error("Qdrant service not available. Cannot perform search.")
            return []  # Return empty list when service is not available

        try:
            # This is a simplified approach - in a real implementation,
            # we might want to use full-text search capabilities of Qdrant
            # or implement custom logic to match the selected text

            # For now, we'll do a regular search but filter results
            # to prioritize those that contain the selected text
            all_results = await self.search_similar(query_embedding, limit=limit*2)

            # Filter and score results based on text similarity to selected text
            filtered_results = []
            for result in all_results:
                text = result["text"].lower()
                selected_lower = selected_text.lower()

                # Simple scoring based on text overlap
                if selected_lower in text:
                    result["score"] *= 1.5  # Boost score for exact matches
                elif any(word in text for word in selected_lower.split()[:3]):  # Check first 3 words
                    result["score"] *= 1.2  # Boost for partial matches

                filtered_results.append(result)

            # Sort by score and return top results
            filtered_results.sort(key=lambda x: x["score"], reverse=True)
            return filtered_results[:limit]

        except Exception as e:
            logger.error(f"Error searching by selected text in Qdrant: {str(e)}")
            raise

    def close(self):
        """Close the Qdrant client"""
        # Qdrant client doesn't have an explicit close method
        pass

# Global instance
qdrant_service = QdrantService()