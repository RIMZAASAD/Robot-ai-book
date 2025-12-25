import pytest
from unittest.mock import Mock, patch, AsyncMock
from src.validation.retrieval_validator import RetrievalValidator
from src.config.validation_config import validation_config
from src.services.vector_search import VectorSearchService
from src.services.metadata_validator import MetadataValidatorService


class TestValidationPipelineIntegration:
    """
    Integration tests for the complete validation pipeline
    """

    @pytest.mark.asyncio
    async def test_complete_validation_pipeline(self):
        """
        Test the complete validation pipeline from query to results
        """
        # Initialize the validator
        validator = RetrievalValidator(validation_config)

        # Mock the external services to avoid actual API calls
        with patch.object(validator.vector_search_service, 'search_similar_chunks') as mock_search, \
             patch.object(validator.metadata_validator, 'validate_batch_metadata') as mock_metadata:

            # Mock search results
            mock_search.return_value = [
                {
                    "id": "chunk_1",
                    "content": "Artificial intelligence is a branch of computer science",
                    "source_url": "https://example.com/ai-intro",
                    "document_id": "doc_123",
                    "score": 0.85,
                    "rank": 1
                },
                {
                    "id": "chunk_2",
                    "content": "Machine learning is a subset of artificial intelligence",
                    "source_url": "https://example.com/ml-overview",
                    "document_id": "doc_456",
                    "score": 0.78,
                    "rank": 2
                }
            ]

            # Mock metadata validation results
            mock_metadata.return_value = {
                "total_chunks": 2,
                "valid_chunks": 2,
                "metadata_accuracy": 1.0,
                "validation_results": [
                    {"metadata_valid": True},
                    {"metadata_valid": True}
                ]
            }

            # Run validation on a sample query
            queries = ["What is artificial intelligence?"]
            report = await validator.run_validation(queries, "integration_test")

            # Verify the results
            assert report.total_queries == 1
            assert report.successful_retrievals == 1
            assert report.precision_score >= 0.8  # Should be high due to good similarity scores
            assert report.metadata_accuracy == 1.0
            assert report.status.value == "completed"

    @pytest.mark.asyncio
    async def test_batch_validation_pipeline(self):
        """
        Test the validation pipeline with multiple queries
        """
        validator = RetrievalValidator(validation_config)

        with patch.object(validator.vector_search_service, 'search_similar_chunks') as mock_search, \
             patch.object(validator.metadata_validator, 'validate_batch_metadata') as mock_metadata:

            # Mock search results for multiple queries
            def mock_search_side_effect(*args, **kwargs):
                return [
                    {
                        "id": f"chunk_{kwargs.get('query_vector', [0])[0] if len(kwargs.get('query_vector', [0])) > 0 else 1}",
                        "content": f"Sample content for query {kwargs.get('limit', 1)}",
                        "source_url": "https://example.com/sample",
                        "document_id": f"doc_{kwargs.get('limit', 1)}",
                        "score": 0.8,
                        "rank": 1
                    }
                ]

            mock_search.return_value = [
                {
                    "id": "chunk_1",
                    "content": "Artificial intelligence concepts explained",
                    "source_url": "https://example.com/ai",
                    "document_id": "doc_1",
                    "score": 0.85,
                    "rank": 1
                }
            ]

            mock_metadata.return_value = {
                "total_chunks": 1,
                "valid_chunks": 1,
                "metadata_accuracy": 1.0,
                "validation_results": [{"metadata_valid": True}]
            }

            # Run validation on multiple queries
            queries = [
                "What is artificial intelligence?",
                "Explain machine learning",
                "How does NLP work?"
            ]

            report = await validator.run_validation(queries, "batch_test")

            # Verify the results
            assert report.total_queries == 3
            assert report.successful_retrievals == 3  # All queries should return results
            assert 0 <= report.precision_score <= 1.0
            assert 0 <= report.metadata_accuracy <= 1.0

    @pytest.mark.asyncio
    async def test_validation_pipeline_with_no_results(self):
        """
        Test the validation pipeline when no results are returned
        """
        validator = RetrievalValidator(validation_config)

        with patch.object(validator.vector_search_service, 'search_similar_chunks') as mock_search, \
             patch.object(validator.metadata_validator, 'validate_batch_metadata') as mock_metadata:

            # Mock empty search results
            mock_search.return_value = []

            # Mock metadata validation (won't be called since no chunks returned)
            mock_metadata.return_value = {
                "total_chunks": 0,
                "valid_chunks": 0,
                "metadata_accuracy": 0.0,
                "validation_results": []
            }

            # Run validation on a query that returns no results
            report = await validator.run_validation(
                ["Query with no matching content"],
                "no_results_test"
            )

            # Verify the results
            assert report.total_queries == 1
            assert report.successful_retrievals == 0  # No successful retrievals
            assert report.precision_score == 0.0  # No relevant results
            assert report.metadata_accuracy == 0.0  # No metadata to validate

    def test_validation_metrics_calculation(self):
        """
        Test the calculation of validation metrics
        """
        validator = RetrievalValidator(validation_config)

        # Simulate validation results
        validation_results = [
            {"is_relevant": True, "relevance_score": 0.85, "metadata_valid": True},
            {"is_relevant": True, "relevance_score": 0.78, "metadata_valid": True},
            {"is_relevant": False, "relevance_score": 0.25, "metadata_valid": False},
            {"is_relevant": True, "relevance_score": 0.92, "metadata_valid": True}
        ]

        metrics = validator._calculate_precision(validation_results)

        # Calculate expected precision: 3 out of 4 are relevant = 0.75
        expected_precision = 0.75
        assert abs(metrics - expected_precision) < 0.001

    @pytest.mark.asyncio
    async def test_error_handling_in_pipeline(self):
        """
        Test error handling in the validation pipeline
        """
        validator = RetrievalValidator(validation_config)

        with patch.object(validator.vector_search_service, 'search_similar_chunks') as mock_search:
            # Mock an exception in the search service
            mock_search.side_effect = Exception("Search service unavailable")

            # Run validation and expect graceful error handling
            result = await validator.validate_single_query("Test query")

            # Verify that error is handled gracefully
            assert "error" in result or result["validation_result"]["relevance_score"] == 0.0
            assert result["validation_result"]["is_relevant"] is False