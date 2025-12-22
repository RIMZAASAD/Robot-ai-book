import os
import logging
import httpx
from typing import List
from transformers import AutoTokenizer, AutoModel
import torch
import numpy as np

logger = logging.getLogger(__name__)

class QwenEmbeddingService:
    def __init__(self):
        # Check if environment variables for OpenRouter embeddings are available
        self.qwen_embedding_api_key = os.getenv("QWEN_EMBEDDING_API_KEY")
        self.qwen_embedding_base_url = os.getenv("QWEN_EMBEDDING_BASE_URL", "https://openrouter.ai/api/v1")
        self.qwen_embedding_model = os.getenv("QWEN_EMBEDDING_MODEL", "nvidia/llama-3.2-nv-embedqa-1b-v1")

        # Initialize attributes for local model fallback
        self.model_name = "sentence-transformers/all-MiniLM-L6-v2"
        self.tokenizer = None
        self.model = None

        # Use API if environment variables are available, otherwise fall back to local model
        if self.qwen_embedding_api_key:
            logger.info("Using OpenRouter embedding API")
            self.use_api = True
        else:
            logger.info("OpenRouter embedding API key not found, falling back to local model")
            self.use_api = False
            # Load local embedding model
            self._load_model()

    def _load_model(self):
        """Load the local embedding model"""
        try:
            self.tokenizer = AutoTokenizer.from_pretrained(self.model_name)
            self.model = AutoModel.from_pretrained(self.model_name)
            logger.info(f"Local embedding model {self.model_name} loaded successfully")
        except Exception as e:
            logger.error(f"Failed to load local embedding model: {e}")
            raise

    async def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for the given texts using either OpenRouter API or local model
        """
        if not texts:
            return []

        if self.use_api:
            return await self._generate_embeddings_via_api(texts)
        else:
            return await self._generate_embeddings_locally(texts)

    async def _generate_embeddings_via_api(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings using OpenRouter API
        """
        headers = {
            "Authorization": f"Bearer {self.qwen_embedding_api_key}",
            "Content-Type": "application/json"
        }

        # Prepare payload for OpenRouter embeddings API
        payload = {
            "model": self.qwen_embedding_model,
            "input": texts
        }

        try:
            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.post(
                    f"{self.qwen_embedding_base_url}/embeddings",
                    json=payload,
                    headers=headers
                )
                response.raise_for_status()

                result = response.json()
                embeddings = [item['embedding'] for item in result['data']]
                logger.info(f"Successfully generated {len(embeddings)} embeddings via OpenRouter API")
                return embeddings
        except Exception as e:
            logger.error(f"Error calling OpenRouter embedding API: {e}")
            logger.info("Falling back to local embedding model")
            # Fall back to local model
            self.use_api = False
            if self.model is None or self.tokenizer is None:  # Only load if not already loaded
                self._load_model()
            return await self._generate_embeddings_locally(texts)

    async def _generate_embeddings_locally(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings using local model
        """
        embeddings = []
        for text in texts:
            try:
                # Tokenize the text
                inputs = self.tokenizer(text, return_tensors="pt", padding=True, truncation=True, max_length=512)

                # Get model output
                with torch.no_grad():
                    outputs = self.model(**inputs)

                # Use mean pooling to get sentence embedding
                embedding = outputs.last_hidden_state.mean(dim=1).squeeze().numpy()
                embeddings.append(embedding.tolist())
            except Exception as e:
                logger.error(f"Error generating local embedding for text: {e}")
                # Add a zero vector as fallback
                embeddings.append([0.0] * 384)  # MiniLM-L6-v2 outputs 384-dim vectors

        return embeddings

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text
        """
        embeddings = await self.generate_embeddings([text])
        return embeddings[0] if embeddings else []

# Global instance
embedding_service = QwenEmbeddingService()