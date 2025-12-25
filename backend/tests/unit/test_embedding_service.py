import pytest
from unittest.mock import Mock, patch
from src.services.embedding_service import EmbeddingService


class TestEmbeddingService:
    """
    Unit tests for the EmbeddingService
    """

    def test_initialization(self):
        """
        Test that the service initializes correctly
        """
        service = EmbeddingService(api_key="test-key", model_name="test-model")
        assert service.api_key == "test-key"
        assert service.model_name == "test-model"

    @pytest.mark.asyncio
    async def test_generate_single_embedding(self):
        """
        Test generating a single embedding
        """
        service = EmbeddingService(api_key="test-key")

        # Mock the Cohere client
        with patch('cohere.Client') as mock_cohere_class:
            mock_client = Mock()
            mock_cohere_class.return_value = mock_client

            # Mock the embed response
            mock_response = Mock()
            mock_response.embeddings = [[0.1, 0.2, 0.3, 0.4]]
            mock_client.embed.return_value = mock_response

            # Test the method
            result = service.generate_single_embedding("test text")

            # Verify the call
            mock_client.embed.assert_called_once_with(
                texts=["test text"],
                model=service.model_name,
                input_type="search_document"
            )

            assert result == [0.1, 0.2, 0.3, 0.4]

    @pytest.mark.asyncio
    async def test_generate_embeddings(self):
        """
        Test generating multiple embeddings
        """
        service = EmbeddingService(api_key="test-key")

        with patch('cohere.Client') as mock_cohere_class:
            mock_client = Mock()
            mock_cohere_class.return_value = mock_client

            # Mock the embed response
            mock_response = Mock()
            mock_response.embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
            mock_client.embed.return_value = mock_response

            # Test the method
            texts = ["test text 1", "test text 2"]
            result = service.generate_embeddings(texts)

            # Verify the call
            mock_client.embed.assert_called_once_with(
                texts=texts,
                model=service.model_name,
                input_type="search_document"
            )

            assert result == [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]

    @pytest.mark.asyncio
    async def test_batch_process_embeddings(self):
        """
        Test batch processing of embeddings
        """
        service = EmbeddingService(api_key="test-key")

        with patch('cohere.Client') as mock_cohere_class:
            mock_client = Mock()
            mock_cohere_class.return_value = mock_client

            # Mock the embed response - return same embedding for each call
            mock_response = Mock()
            mock_response.embeddings = [[0.1, 0.2, 0.3]]
            mock_client.embed.return_value = mock_response

            # Test with batch size of 2 and 3 texts
            texts = ["text1", "text2", "text3"]
            result = service.batch_process_embeddings(texts, batch_size=2)

            # Should have been called twice (once for [text1, text2], once for [text3])
            assert mock_client.embed.call_count == 2
            assert len(result) == 3

    def test_execute_method(self):
        """
        Test the execute method
        """
        service = EmbeddingService(api_key="test-key")

        with patch.object(service, 'generate_embeddings', return_value=[[0.1, 0.2, 0.3]]) as mock_gen:
            result = asyncio.run(service.execute(["test text"]))

            mock_gen.assert_called_once_with(["test text"])
            assert result == [[0.1, 0.2, 0.3]]