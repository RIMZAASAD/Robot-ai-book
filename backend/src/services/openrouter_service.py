import os
import httpx
import asyncio
import logging
from typing import List, Dict, Any
from tenacity import retry, stop_after_attempt, wait_exponential

logger = logging.getLogger(__name__)

class OpenRouterService:
    def __init__(self):
        self.api_key = os.getenv("OPENROUTER_API_KEY")

        if self.api_key:
            logger.info("OpenRouter API key found, initializing service")
            self.is_available = True
            self.base_url = "https://openrouter.ai/api/v1"
            self.client = httpx.AsyncClient(
                headers={
                    "Authorization": f"Bearer {self.api_key}",
                    "Content-Type": "application/json"
                },
                timeout=30.0
            )
        else:
            logger.warning("OPENROUTER_API_KEY environment variable not set, service will not be available")
            self.is_available = False
            self.client = None

    @retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=4, max=10))
    async def generate_response(self,
                              messages: List[Dict[str, str]],
                              model: str = "openai/gpt-4o",
                              temperature: float = 0.1,
                              max_tokens: int = 1000) -> str:
        """
        Generate response using OpenRouter API
        """
        if not self.is_available:
            error_msg = "OpenRouter API key not configured. Please set OPENROUTER_API_KEY environment variable."
            logger.error(error_msg)
            return f"Error: {error_msg}"

        payload = {
            "model": model,
            "messages": messages,
            "temperature": temperature,
            "max_tokens": max_tokens
        }

        try:
            response = await self.client.post(
                f"{self.base_url}/chat/completions",
                json=payload
            )
            response.raise_for_status()

            result = response.json()
            return result["choices"][0]["message"]["content"]

        except httpx.HTTPStatusError as e:
            logger.error(f"OpenRouter API error: {e.response.status_code} - {e.response.text}")
            raise
        except Exception as e:
            logger.error(f"Unexpected error calling OpenRouter API: {str(e)}")
            raise

    async def close(self):
        """Close the HTTP client"""
        if self.client:
            await self.client.aclose()

# Global instance
openrouter_service = OpenRouterService()