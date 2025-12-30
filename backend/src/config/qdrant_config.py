from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from typing import Optional
import logging
from .settings import settings


class QdrantConfig:
    """
    Configuration and initialization for Qdrant vector database
    """

    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.client = None
        self.collection_name = settings.collection_name

    def initialize_client(self) -> QdrantClient:
        """
        Initialize Qdrant client based on configuration
        """
        if settings.qdrant_url and settings.qdrant_api_key:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                timeout=10.0
            )
            self.logger.info("Initialized Qdrant client with cloud configuration")
        else:
            # Fallback to local Qdrant
            self.client = QdrantClient(
                host=settings.qdrant_host,
                port=settings.qdrant_port
            )
            self.logger.info(f"Initialized Qdrant client with local configuration: {settings.qdrant_host}:{settings.qdrant_port}")

        return self.client

    def initialize_collection(self, vector_size: int = 1024):
        """
        Initialize the Qdrant collection with proper schema
        """
        if not self.client:
            raise ValueError("Qdrant client not initialized. Call initialize_client() first.")

        try:
            collections = self.client.get_collections()
            collection_exists = any(col.name == self.collection_name for col in collections.collections)

            if not collection_exists:
                # Create collection - Cohere embeddings are 1024-dimensional
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=vector_size, distance=Distance.COSINE)
                )
                self.logger.info(f"Created Qdrant collection: {self.collection_name} with {vector_size} dimensions")
            else:
                self.logger.info(f"Qdrant collection {self.collection_name} already exists")
        except Exception as e:
            self.logger.error(f"Error initializing Qdrant collection: {str(e)}")
            raise

    def get_client(self) -> QdrantClient:
        """
        Get the initialized Qdrant client
        """
        if not self.client:
            self.initialize_client()
        return self.client

    def health_check(self) -> dict:
        """
        Perform a health check on the Qdrant connection
        """
        try:
            client = self.get_client()
            info = client.info()
            return {
                "status": "healthy",
                "qdrant_info": info,
                "collection_name": self.collection_name
            }
        except Exception as e:
            self.logger.error(f"Qdrant health check failed: {str(e)}")
            return {
                "status": "unhealthy",
                "error": str(e)
            }