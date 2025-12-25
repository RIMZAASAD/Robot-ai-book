import pytest
from unittest.mock import Mock, patch, AsyncMock
from src.validation.retrieval_validator import RetrievalValidator
from src.config.validation_config import validation_config


class TestRetrievalValidator:
    """
    Unit tests for the RetrievalValidator class
    """

    def test_initialization(self):
        """
        Test that the validator initializes correctly with config
        """
        validator = RetrievalValidator(validation_config)
        assert validator.config == validation_config

    @pytest.mark.asyncio
    async def test_validate_single_query(self):
        """
        Test validating a single query
        """
        validator = RetrievalValidator(validation_config)

        # Mock the query processing and search services
        with patch.object(validator, '_process_query', new_callable=AsyncMock) as mock_process, \
             patch.object(validator, '_validate_relevance', return_value=True) as mock_validate:

            mock_process.return_value = {
                "query_id": "test_query_1",
                "retrieved_chunks": [
                    {
                        "id": "chunk_1",
                        "content": "Test content for validation",
                        "source_url": "https://example.com",
                        "document_id": "doc_1",
                        "score": 0.85,
                        "rank": 1
                    }
                ],
                "processing_time": 0.1
            }

            result = await validator.validate_single_query("What is AI?")

            assert result is not None
            assert "validation_result" in result
            mock_process.assert_called_once()
            mock_validate.assert_called_once()

    @pytest.mark.asyncio
    async def test_validate_batch_queries(self):
        """
        Test validating multiple queries in batch
        """
        validator = RetrievalValidator(validation_config)

        sample_queries = ["What is AI?", "Explain ML?"]

        with patch.object(validator, 'validate_single_query', new_callable=AsyncMock) as mock_single:
            mock_single.return_value = {
                "query": "Test query",
                "validation_result": {"is_relevant": True, "relevance_score": 0.85}
            }

            results = await validator.validate_batch_queries(sample_queries)

            assert len(results) == len(sample_queries)
            assert mock_single.call_count == len(sample_queries)

    def test_validate_relevance(self):
        """
        Test relevance validation logic
        """
        validator = RetrievalValidator(validation_config)

        # Test with high similarity score
        result = validator._validate_relevance(0.85, validation_config.validation_threshold)
        assert result is True

        # Test with low similarity score
        result = validator._validate_relevance(0.1, validation_config.validation_threshold)
        assert result is False

    def test_calculate_precision(self):
        """
        Test precision calculation
        """
        validator = RetrievalValidator(validation_config)

        validation_results = [
            {"is_relevant": True},
            {"is_relevant": True},
            {"is_relevant": False},
            {"is_relevant": True}
        ]

        precision = validator._calculate_precision(validation_results)
        assert precision == 0.75  # 3 out of 4 are relevant