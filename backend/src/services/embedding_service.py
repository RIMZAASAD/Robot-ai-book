try:
    import cohere
    _HAS_COHERE = True
except Exception:
    cohere = None
    _HAS_COHERE = False

from typing import List, Dict, Any
from .base_service import BaseService
from ..config.settings import settings


class EmbeddingService(BaseService):
    """
    Service for generating embeddings using Cohere (falls back to a dummy generator when Cohere is not available)
    """

    def __init__(self, api_key: str = None, model_name: str = None):
        super().__init__()
        self.api_key = api_key or settings.cohere_api_key
        self.model_name = model_name or settings.cohere_model
        if _HAS_COHERE:
            self.client = cohere.Client(self.api_key)
        else:
            self.client = None
            self.logger.warning("Cohere SDK not available; EmbeddingService will use a dummy embedding generator for local development.")

    def generate_embeddings(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere
        """
        if not self.client:
            # Fallback for local development when Cohere SDK is missing
            self.logger.warning("Cohere client not available; generating dummy embeddings for local development")
            dummy_len = 512
            embeddings = [[0.0] * dummy_len for _ in texts]
            self.logger.info(f"Generated {len(embeddings)} dummy embeddings (length={dummy_len})")
            return embeddings

        try:
            response = self.client.embed(
                texts=texts,
                model=self.model_name,
                input_type=input_type
            )
            
            # Using .embeddings which is available in newer cohere-python versions
            embeddings = [embedding for embedding in response.embeddings]
            self.logger.info(f"Generated {len(embeddings)} embeddings with input_type={input_type}")
            return embeddings
        except Exception as e:
            error_result = self.handle_error(e, "Embedding generation")
            raise

    def generate_single_embedding(self, text: str, input_type: str = "search_document") -> List[float]:
        """
        Generate a single embedding for a text
        """
        return self.generate_embeddings([text], input_type=input_type)[0]

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

    async def execute(self, texts: List[str], input_type: str = "search_document") -> List[List[float]]:
        """
        Execute the embedding generation operation
        """
        return self.generate_embeddings(texts, input_type=input_type)