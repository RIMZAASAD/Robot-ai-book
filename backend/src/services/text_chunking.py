from typing import List, Dict, Any
from .base_service import BaseService
import re


class TextChunkingService(BaseService):
    """
    Service for chunking text into semantically meaningful segments
    """

    def __init__(self, default_chunk_size: int = 512, default_overlap: int = 102):
        super().__init__()
        self.default_chunk_size = default_chunk_size
        self.default_overlap = default_overlap

    def chunk_text(self, text: str, chunk_size: int = None, overlap: int = None) -> List[Dict[str, Any]]:
        """
        Chunk text into segments with overlap
        """
        if chunk_size is None:
            chunk_size = self.default_chunk_size
        if overlap is None:
            overlap = self.default_overlap

        try:
            # Split content into sentences to maintain semantic boundaries
            sentences = re.split(r'[.!?]+\s+', text)
            chunks = []
            current_chunk = ""
            chunk_index = 0

            for sentence in sentences:
                sentence = sentence.strip()
                if not sentence:
                    continue

                # Check if adding this sentence would exceed chunk size
                if len(current_chunk) + len(sentence) <= chunk_size:
                    current_chunk += sentence + ". "
                else:
                    # If current chunk is not empty, save it
                    if current_chunk.strip():
                        chunks.append({
                            "id": f"chunk_{chunk_index}",
                            "content": current_chunk.strip(),
                            "chunk_index": chunk_index,
                            "original_position": len(chunks),
                            "size": len(current_chunk),
                            "sentence_count": current_chunk.count('. ')
                        })
                        chunk_index += 1

                    # Start new chunk with overlap from previous chunk if possible
                    # For now, we'll just start with the current sentence
                    current_chunk = sentence + ". "

            # Add the last chunk if it has content
            if current_chunk.strip():
                chunks.append({
                    "id": f"chunk_{chunk_index}",
                    "content": current_chunk.strip(),
                    "chunk_index": chunk_index,
                    "original_position": len(chunks),
                    "size": len(current_chunk),
                    "sentence_count": current_chunk.count('. ')
                })

            return chunks
        except Exception as e:
            error_result = self.handle_error(e, "Text chunking")
            return []

    def chunk_by_tokens(self, text: str, max_tokens: int = 512, overlap_tokens: int = 102) -> List[Dict[str, Any]]:
        """
        Chunk text based on token count (simplified tokenization)
        """
        try:
            # Simple tokenization by splitting on whitespace
            words = text.split()
            chunks = []
            chunk_index = 0

            i = 0
            while i < len(words):
                # Take a chunk of max_tokens words
                chunk_words = words[i:i + max_tokens]
                chunk_text = " ".join(chunk_words)

                chunks.append({
                    "id": f"token_chunk_{chunk_index}",
                    "content": chunk_text,
                    "chunk_index": chunk_index,
                    "word_count": len(chunk_words),
                    "original_position": i
                })

                # Move to the next chunk, considering overlap
                i += max_tokens - overlap_tokens
                chunk_index += 1

            return chunks
        except Exception as e:
            error_result = self.handle_error(e, "Token-based text chunking")
            return []

    def validate_chunk_parameters(self, chunk_size: int, overlap: int) -> bool:
        """
        Validate chunking parameters
        """
        return chunk_size > 0 and overlap >= 0 and overlap < chunk_size

    async def execute(self, text: str, chunk_size: int = None, overlap: int = None) -> List[Dict[str, Any]]:
        """
        Execute the text chunking operation
        """
        return self.chunk_text(text, chunk_size, overlap)