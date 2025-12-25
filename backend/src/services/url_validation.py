import re
from urllib.parse import urlparse
from typing import Union
from ..models.document import Document


class URLValidationService:
    """
    Service for validating URLs before processing
    """

    @staticmethod
    def is_valid_url(url: str) -> bool:
        """
        Check if a URL is valid
        """
        try:
            result = urlparse(url)
            return all([result.scheme, result.netloc]) and result.scheme in ['http', 'https']
        except Exception:
            return False

    @staticmethod
    def normalize_url(url: str) -> str:
        """
        Normalize URL by ensuring it has proper scheme
        """
        if not url.startswith(('http://', 'https://')):
            url = 'https://' + url
        return url

    @staticmethod
    def validate_and_clean_urls(urls: list) -> list:
        """
        Validate and clean a list of URLs
        """
        valid_urls = []
        for url in urls:
            normalized_url = URLValidationService.normalize_url(url.strip())
            if URLValidationService.is_valid_url(normalized_url):
                valid_urls.append(normalized_url)
        return valid_urls