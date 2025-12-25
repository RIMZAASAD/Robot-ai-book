import re
from typing import Dict, Any
from .base_service import BaseService


class TextCleaningService(BaseService):
    """
    Service for cleaning and normalizing extracted text content
    """

    def __init__(self):
        super().__init__()

    def clean_text(self, text: str) -> Dict[str, Any]:
        """
        Clean and normalize text content
        """
        try:
            # Remove extra whitespace
            cleaned_text = re.sub(r'\s+', ' ', text)

            # Remove special characters and normalize
            cleaned_text = re.sub(r'[^\w\s\.\!\?,:;\'\"-]', ' ', cleaned_text)

            # Remove extra spaces around punctuation
            cleaned_text = re.sub(r'\s+([.,!?;:])', r'\1', cleaned_text)

            # Remove extra spaces
            cleaned_text = cleaned_text.strip()

            # Remove duplicate sentences (optional)
            sentences = cleaned_text.split('. ')
            unique_sentences = []
            seen = set()

            for sentence in sentences:
                sentence = sentence.strip()
                if sentence and sentence not in seen:
                    seen.add(sentence)
                    unique_sentences.append(sentence)

            deduplicated_text = '. '.join(unique_sentences)

            return {
                "cleaned_text": deduplicated_text,
                "original_length": len(text),
                "cleaned_length": len(deduplicated_text),
                "success": True
            }
        except Exception as e:
            error_result = self.handle_error(e, "Text cleaning")
            error_result["success"] = False
            error_result["cleaned_text"] = text  # Return original text on error
            error_result["original_length"] = len(text)
            error_result["cleaned_length"] = len(text)
            return error_result

    def remove_boilerplate(self, text: str) -> str:
        """
        Remove common boilerplate content from text
        """
        # Common boilerplate patterns to remove
        boilerplate_patterns = [
            r'copyright\s+\d{4}',  # Copyright notices
            r'all\s+rights\s+reserved',  # Rights reserved
            r'privacy\s+policy',  # Privacy policy links
            r'terms\s+of\s+use',  # Terms of use
            r'contact\s+us',  # Contact us links
            r'menu',  # Menu items
            r'home\s+about',  # Navigation
        ]

        cleaned_text = text
        for pattern in boilerplate_patterns:
            cleaned_text = re.sub(pattern, '', cleaned_text, flags=re.IGNORECASE)

        return cleaned_text.strip()

    async def execute(self, text: str) -> Dict[str, Any]:
        """
        Execute the text cleaning operation
        """
        return self.clean_text(text)