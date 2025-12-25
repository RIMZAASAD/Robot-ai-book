import pytest
from main import URLIngestionService, TextChunkingService

def test_url_extraction():
    # This would test URL extraction, but we'll mock for now since we need a real URL
    pass

def test_text_chunking():
    sample_text = "This is a sample text. It has multiple sentences. Each should be handled properly."
    chunks = TextChunkingService.chunk_text(sample_text, chunk_size=50, overlap=10)

    assert len(chunks) > 0
    assert "content" in chunks[0]
    assert "chunk_index" in chunks[0]