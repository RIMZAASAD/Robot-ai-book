from typing import Union
import re


class TokenCounter:
    """
    Utility for counting tokens in text content
    """

    @staticmethod
    def count_tokens(text: str, method: str = "words") -> int:
        """
        Count tokens in text using specified method
        """
        if method == "words":
            # Simple word-based tokenization
            if not text:
                return 0
            words = text.split()
            return len(words)
        elif method == "characters":
            return len(text)
        elif method == "sentences":
            sentences = re.split(r'[.!?]+', text)
            return len([s for s in sentences if s.strip()])
        else:
            raise ValueError(f"Unknown tokenization method: {method}")

    @staticmethod
    def estimate_cohere_tokens(text: str) -> int:
        """
        Estimate the number of tokens for Cohere models
        (Cohere uses a specific tokenization method)
        """
        # As a rough estimate, divide character count by 4 (common approximation)
        # For more accurate results, Cohere's tokenizer would be needed
        if not text:
            return 0
        return len(text) // 4

    @staticmethod
    def validate_chunk_size(text: str, max_tokens: int, method: str = "words") -> bool:
        """
        Validate if text is within token limit
        """
        token_count = TokenCounter.count_tokens(text, method)
        return token_count <= max_tokens