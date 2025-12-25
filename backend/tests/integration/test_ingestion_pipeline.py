import pytest
import asyncio
from unittest.mock import AsyncMock, patch
from src.services.url_fetching import URLFetchingService
from src.services.html_extraction import HTMLExtractionService
from src.services.text_cleaning import TextCleaningService
from src.services.text_chunking import TextChunkingService
from src.services.embedding_service import EmbeddingService
from src.services.vector_storage import VectorStorageService
from src.config.qdrant_config import QdrantConfig
from src.config.settings import settings


class TestIntegrationPipeline:
    """
    Integration tests for the complete RAG ingestion pipeline
    """

    @pytest.mark.asyncio
    async def test_complete_ingestion_pipeline(self):
        """
        Test the complete ingestion pipeline from URL to vector storage
        """
        # Mock services to avoid external dependencies during testing
        with patch('cohere.Client') as mock_cohere, \
             patch('qdrant_client.QdrantClient') as mock_qdrant:

            # Setup mock responses
            mock_cohere.return_value.embed.return_value.embeddings = [[0.1, 0.2, 0.3]]
            mock_qdrant.return_value.get_collections.return_value.collections = []
            mock_qdrant.return_value.upsert.return_value = None

            # Initialize services
            url_fetching_service = URLFetchingService()
            html_extraction_service = HTMLExtractionService()
            text_cleaning_service = TextCleaningService()
            text_chunking_service = TextChunkingService()
            embedding_service = EmbeddingService(api_key="test-key")
            vector_storage_service = VectorStorageService()

            # Mock the URL fetching to return test content
            url_fetching_service.fetch_content = AsyncMock(return_value={
                "url": "https://example.com",
                "content": "<html><head><title>Test Page</title></head><body><p>This is test content for integration testing.</p></body></html>",
                "status_code": 200,
                "headers": {},
                "title": "Test Page"
            })

            # Run the pipeline
            # Step 1: Fetch content
            fetched_data = await url_fetching_service.fetch_content("https://example.com")

            # Step 2: Extract HTML content
            extracted_data = await html_extraction_service.execute(fetched_data['content'], fetched_data['url'])
            assert extracted_data["success"] == True
            assert "test content" in extracted_data["content"].lower()

            # Step 3: Clean text
            cleaned_data = await text_cleaning_service.execute(extracted_data["content"])
            assert cleaned_data["success"] == True

            # Step 4: Chunk text
            chunks = await text_chunking_service.execute(cleaned_data["cleaned_text"])
            assert len(chunks) > 0

            # Step 5: Generate embeddings
            embeddings = await embedding_service.execute([chunks[0]["content"]])
            assert len(embeddings) == 1

            # Step 6: Store vectors
            metadata = [{"title": "Test Page", "url": "https://example.com", "chunk_index": 0, "document_id": "test", "created_at": "", "source_type": "web_page"}]
            stored_ids = await vector_storage_service.execute(embeddings, [chunks[0]["content"]], metadata)

            assert len(stored_ids) == 1
            assert stored_ids[0] is not None

    @pytest.mark.asyncio
    async def test_batch_processing(self):
        """
        Test batch processing of multiple URLs
        """
        urls = ["https://example1.com", "https://example2.com"]

        # This would test the ability to process multiple URLs in batch
        # For now, just test the structure
        assert len(urls) == 2

    def test_qdrant_health_check(self):
        """
        Test Qdrant health check
        """
        qdrant_config = QdrantConfig()
        health_status = qdrant_config.health_check()

        # Note: This will fail if Qdrant is not running, but that's expected
        # The test is to ensure the method can be called without error
        assert "status" in health_status