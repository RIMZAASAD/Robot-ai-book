from bs4 import BeautifulSoup
from typing import Dict, Any
from .base_service import BaseService


class HTMLExtractionService(BaseService):
    """
    Service for extracting clean text content from HTML
    """

    def __init__(self):
        super().__init__()

    def extract_content(self, html_content: str, url: str = "") -> Dict[str, Any]:
        """
        Extract clean text content from HTML
        """
        try:
            soup = BeautifulSoup(html_content, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style", "nav", "header", "footer", "aside"]):
                script.decompose()

            # Extract title
            title_tag = soup.find('title')
            title = title_tag.get_text().strip() if title_tag else "No Title"

            # Get main content - prioritize main content areas
            main_content = soup.find('main') or soup.find('article') or soup.find('div', class_='content') or soup

            # Extract text content
            text = main_content.get_text()

            # Clean up text
            lines = (line.strip() for line in text.splitlines())
            chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
            clean_text = ' '.join(chunk for chunk in chunks if chunk)

            return {
                "url": url,
                "title": title,
                "content": clean_text,
                "word_count": len(clean_text.split()),
                "success": True
            }
        except Exception as e:
            error_result = self.handle_error(e, f"HTML extraction for {url}")
            error_result["success"] = False
            error_result["url"] = url
            error_result["title"] = "Error Extracting Title"
            error_result["content"] = ""
            error_result["word_count"] = 0
            return error_result

    async def execute(self, html_content: str, url: str = "") -> Dict[str, Any]:
        """
        Execute the HTML extraction operation
        """
        return self.extract_content(html_content, url)