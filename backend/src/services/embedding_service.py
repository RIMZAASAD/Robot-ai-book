import cohere
from typing import List, Dict, Any
from .base_service import BaseService
from ..config.settings import settings


class EmbeddingService(BaseService):
    """
    Service for generating embeddings using Cohere
    """

    def __init__(self, api_key: str = None, model_name: str = None):
        super().__init__()
        self.api_key = api_key or settings.cohere_api_key
        self.model_name = model_name or settings.cohere_model
        self.client = cohere.Client(self.api_key)

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere
        """
        try:
            response = self.client.embed(
                texts=texts,
                model=self.model_name,
                input_type="search_document"
            )

            embeddings = [embedding for embedding in response.embeddings]
            self.logger.info(f"Generated {len(embeddings)} embeddings")
            return embeddings
        except Exception as e:
            error_result = self.handle_error(e, "Embedding generation")
            raise

    def generate_single_embedding(self, text: str) -> List[float]:
        """
        Generate a single embedding for a text
        """
        try:
            response = self.client.embed(
                texts=[text],
                model=self.model_name,
                input_type="search_document"
            )

            embedding = response.embeddings[0]
            self.logger.info(f"Generated single embedding")
            return embedding
        except Exception as e:
            error_result = self.handle_error(e, "Single embedding generation")
            raise

    def batch_process_embeddings(self, texts: List[str], batch_size: int = 50) -> List[List[float]]:
        """
        Process embeddings in batches to handle large inputs
        """
        all_embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]
            batch_embeddings = self.generate_embeddings(batch)
            all_embeddings.extend(batch_embeddings)

        return all_embeddings

    async def execute(self, texts: List[str]) -> List[List[float]]:
        """
        Execute the embedding generation operation
        """
        return self.generate_embeddings(texts)