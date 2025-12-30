import pytest
from src.services.text_chunking import TextChunkingService

@pytest.mark.asyncio
async def test_text_chunking():
    sample_text = "This is a sample text. It has multiple sentences. Each should be handled properly."
    service = TextChunkingService()
    # Note: text_chunking.py execute method doesn't take chunk_size/overlap as args, 
    # it uses settings which are loaded in __init__ or from config.
    chunks = await service.execute(sample_text)

    assert len(chunks) > 0
    assert "content" in chunks[0]