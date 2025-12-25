import pytest
from unittest.mock import Mock, patch, AsyncMock
from src.validation.query_processor import QueryProcessor
from src.config.validation_config import validation_config


class TestQueryProcessor:
    """
    Unit tests for the QueryProcessor class
    """

    def test_initialization(self):
        """
        Test that the query processor initializes correctly with config
        """
        processor = QueryProcessor(validation_config)
        assert processor.config == validation_config

    @pytest.mark.asyncio
    async def test_process_single_query(self):
        """
        Test processing a single query for embedding generation
        """
        processor = QueryProcessor(validation_config)

        # Mock the embedding service
        with patch.object(processor.embedding_service, 'execute', new_callable=AsyncMock) as mock_execute:
            mock_execute.return_value = [[0.1, 0.2, 0.3, 0.4]]  # Mock embedding response

            result = await processor.process_query("Test query")

            assert result is not None
            assert "query_embedding" in result
            assert result["embedding_dimension"] == 4  # Length of mock embedding
            assert result["query_text"] == "Test query"
            mock_execute.assert_called_once_with(["Test query"])

    @pytest.mark.asyncio
    async def test_process_batch_queries(self):
        """
        Test processing multiple queries in batch
        """
        processor = QueryProcessor(validation_config)

        sample_queries = ["Query 1", "Query 2", "Query 3"]

        with patch.object(processor, 'process_query', new_callable=AsyncMock) as mock_process:
            mock_process.return_value = {
                "query_text": "Test query",
                "query_embedding": [0.1, 0.2, 0.3],
                "embedding_dimension": 3
            }

            results = await processor.process_batch_queries(sample_queries)

            assert len(results) == len(sample_queries)
            assert mock_process.call_count == len(sample_queries)

    def test_validate_embedding_correct_format(self):
        """
        Test embedding validation with correct format and dimensions
        """
        processor = QueryProcessor(validation_config)

        # Test with correct dimension (1024)
        valid_embedding = [0.1] * 1024
        result = processor.validate_embedding(valid_embedding)

        assert result["is_valid"] is True
        assert result["dimension_valid"] is True
        assert result["format_valid"] is True
        assert len(result["errors"]) == 0

    def test_validate_embedding_wrong_dimension(self):
        """
        Test embedding validation with wrong dimension
        """
        processor = QueryProcessor(validation_config)

        # Test with wrong dimension (100 instead of 1024)
        invalid_embedding = [0.1] * 100
        result = processor.validate_embedding(invalid_embedding)

        assert result["is_valid"] is False
        assert result["dimension_valid"] is False
        assert "Dimension mismatch" in result["errors"][0]

    def test_validate_embedding_wrong_format(self):
        """
        Test embedding validation with wrong format
        """
        processor = QueryProcessor(validation_config)

        # Test with non-list input
        result = processor.validate_embedding("not a list")
        assert result["is_valid"] is False
        assert result["format_valid"] is False

        # Test with non-numeric values
        result = processor.validate_embedding([0.1, "not a number", 0.3])
        assert result["is_valid"] is False
        assert result["format_valid"] is False

    def test_compare_embeddings_similarity(self):
        """
        Test comparing embeddings for similarity
        """
        processor = QueryProcessor(validation_config)

        # Two identical embeddings should have similarity of 1.0
        emb1 = [1.0, 0.0, 0.0]
        emb2 = [1.0, 0.0, 0.0]
        result = processor.compare_embeddings(emb1, emb2)

        assert result["consistent"] is True  # Assuming threshold is low enough
        assert abs(result["similarity"] - 1.0) < 0.001

        # Two orthogonal embeddings should have similarity of 0.0
        emb1 = [1.0, 0.0, 0.0]
        emb2 = [0.0, 1.0, 0.0]
        result = processor.compare_embeddings(emb1, emb2)

        assert abs(result["similarity"] - 0.0) < 0.001

    def test_cosine_similarity_calculation(self):
        """
        Test cosine similarity calculation
        """
        processor = QueryProcessor(validation_config)

        # Test with known vectors
        emb1 = [1.0, 0.0]
        emb2 = [0.0, 1.0]
        similarity = processor._cosine_similarity(emb1, emb2)
        assert abs(similarity - 0.0) < 0.001

        # Test with same vectors
        emb1 = [1.0, 0.0]
        emb2 = [1.0, 0.0]
        similarity = processor._cosine_similarity(emb1, emb2)
        assert abs(similarity - 1.0) < 0.001

    def test_semantic_consistency_validation(self):
        """
        Test semantic consistency validation between embeddings
        """
        processor = QueryProcessor(validation_config)

        # Test with similar embeddings
        query_embeddings = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
        stored_embeddings = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]

        result = processor.validate_semantic_consistency(query_embeddings, stored_embeddings)

        assert result["consistent"] is True
        assert result["average_similarity"] >= 0.9  # Should be high for identical embeddings
        assert result["total_comparisons"] == 4  # 2 query * 2 stored = 4 comparisons