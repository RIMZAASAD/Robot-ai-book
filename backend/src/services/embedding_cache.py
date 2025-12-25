from typing import Dict, List, Optional, Any
import hashlib
import json
from datetime import datetime, timedelta
import logging


class EmbeddingCache:
    """
    Service for caching embeddings to avoid redundant API calls
    """

    def __init__(self, ttl_minutes: int = 1440):  # 24 hours default TTL
        self.cache: Dict[str, Dict[str, Any]] = {}
        self.ttl_minutes = ttl_minutes
        self.logger = logging.getLogger(self.__class__.__name__)

    def _generate_key(self, text: str, model_name: str = "default") -> str:
        """
        Generate a cache key for the text and model
        """
        key_string = f"{model_name}:{text}"
        return hashlib.md5(key_string.encode()).hexdigest()

    def get(self, text: str, model_name: str = "default") -> Optional[List[float]]:
        """
        Get cached embedding if it exists and is not expired
        """
        key = self._generate_key(text, model_name)

        if key in self.cache:
            cached_data = self.cache[key]
            if datetime.now() < cached_data["expires_at"]:
                self.logger.info(f"Cache hit for text: {text[:50]}...")
                return cached_data["embedding"]
            else:
                # Remove expired entry
                del self.cache[key]
                self.logger.info(f"Cache entry expired for text: {text[:50]}...")

        return None

    def set(self, text: str, embedding: List[float], model_name: str = "default"):
        """
        Store embedding in cache
        """
        key = self._generate_key(text, model_name)
        expires_at = datetime.now() + timedelta(minutes=self.ttl_minutes)

        self.cache[key] = {
            "embedding": embedding,
            "text": text,
            "model_name": model_name,
            "created_at": datetime.now(),
            "expires_at": expires_at
        }

        self.logger.info(f"Cache set for text: {text[:50]}...")

    def clear_expired(self):
        """
        Clear all expired cache entries
        """
        current_time = datetime.now()
        expired_keys = [
            key for key, data in self.cache.items()
            if current_time >= data["expires_at"]
        ]

        for key in expired_keys:
            del self.cache[key]

        self.logger.info(f"Cleared {len(expired_keys)} expired cache entries")

    def clear_all(self):
        """
        Clear all cache entries
        """
        self.cache.clear()
        self.logger.info("Cleared all cache entries")

    def get_stats(self) -> Dict[str, Any]:
        """
        Get cache statistics
        """
        total = len(self.cache)
        expired = sum(1 for data in self.cache.values() if datetime.now() >= data["expires_at"])

        return {
            "total_entries": total,
            "expired_entries": expired,
            "valid_entries": total - expired,
            "ttl_minutes": self.ttl_minutes
        }