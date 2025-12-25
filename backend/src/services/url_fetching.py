import httpx
import asyncio
from typing import Dict, Any, Optional
from urllib.parse import urljoin, urlparse
import logging
from .base_service import BaseService
from .url_validation import URLValidationService


class URLFetchingService(BaseService):
    """
    Service for fetching content from URLs
    """

    def __init__(self, timeout: int = 30, max_retries: int = 3):
        super().__init__()
        self.timeout = timeout
        self.max_retries = max_retries
        self.client = httpx.AsyncClient(timeout=timeout)

    async def fetch_content(self, url: str) -> Dict[str, Any]:
        """
        Fetch content from a URL with retry logic
        """
        if not URLValidationService.is_valid_url(url):
            raise ValueError(f"Invalid URL: {url}")

        for attempt in range(self.max_retries):
            try:
                response = await self.client.get(
                    url,
                    headers={
                        'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
                    }
                )
                response.raise_for_status()

                return {
                    "url": url,
                    "content": response.text,
                    "status_code": response.status_code,
                    "headers": dict(response.headers),
                    "title": self._extract_title(response.text)
                }
            except httpx.RequestError as e:
                self.logger.warning(f"Attempt {attempt + 1} failed for {url}: {str(e)}")
                if attempt == self.max_retries - 1:
                    raise
                await asyncio.sleep(2 ** attempt)  # Exponential backoff
            except httpx.HTTPStatusError as e:
                self.logger.warning(f"HTTP error {e.response.status_code} for {url}")
                raise

    def _extract_title(self, html_content: str) -> str:
        """
        Extract title from HTML content
        """
        import re
        title_match = re.search(r'<title[^>]*>(.*?)</title>', html_content, re.IGNORECASE | re.DOTALL)
        if title_match:
            import html
            return html.unescape(title_match.group(1)).strip()
        return "No Title"

    async def close(self):
        """
        Close the HTTP client
        """
        await self.client.aclose()

    async def execute(self, url: str) -> Dict[str, Any]:
        """
        Execute the URL fetching operation
        """
        return await self.fetch_content(url)