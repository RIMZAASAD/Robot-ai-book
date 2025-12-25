import pytest
from src.services.metadata_validator import MetadataValidatorService


class TestRetrievalAccuracy:
    """
    Validation tests for retrieval accuracy and metadata preservation
    """

    def test_metadata_preservation_validation(self):
        """
        Test validation of metadata preservation in retrieved chunks
        """
        validator = MetadataValidatorService()

        retrieved_chunk = {
            "id": "chunk_123",
            "content": "Sample content retrieved from source",
            "source_url": "https://example.com/document",
            "document_id": "doc_456",
            "score": 0.85,
            "rank": 1
        }

        validation_result = validator.validate_metadata(retrieved_chunk)

        assert validation_result["metadata_valid"] is True
        assert "source_url" in retrieved_chunk
        assert "document_id" in retrieved_chunk
        assert len(validation_result["missing_fields"]) == 0

    def test_source_url_document_id_validation(self):
        """
        Test validation of source URL and document ID specifically
        """
        validator = MetadataValidatorService()

        # Test with valid URL and document ID
        valid_chunk = {
            "source_url": "https://wikipedia.org/article",
            "document_id": "wiki_123",
            "content": "Valid content"
        }

        result = validator.validate_metadata(valid_chunk)
        assert result["metadata_valid"] is True

        # Test with invalid URL
        invalid_url_chunk = {
            "source_url": "invalid_url",
            "document_id": "doc_123",
            "content": "Content with invalid URL"
        }

        result = validator.validate_metadata(invalid_url_chunk)
        assert result["metadata_valid"] is False
        assert "source_url" in result["incorrect_fields"]

        # Test with missing document ID
        missing_doc_chunk = {
            "source_url": "https://example.com",
            # Missing document_id
            "content": "Content without document ID"
        }

        result = validator.validate_metadata(missing_doc_chunk)
        assert result["metadata_valid"] is False
        assert "document_id" in result["missing_fields"]

    def test_metadata_completeness_validation(self):
        """
        Test validation of metadata completeness across multiple chunks
        """
        validator = MetadataValidatorService()

        retrieved_chunks = [
            {
                "id": "chunk_1",
                "content": "Content 1",
                "source_url": "https://example.com/doc1",
                "document_id": "doc_1"
            },
            {
                "id": "chunk_2",
                "content": "Content 2",
                "source_url": "https://example.com/doc2",
                "document_id": "doc_2"
            }
        ]

        batch_result = validator.validate_batch_metadata(retrieved_chunks)

        assert batch_result["total_chunks"] == 2
        assert batch_result["valid_chunks"] == 2
        assert batch_result["metadata_accuracy"] == 1.0

        # Test with some invalid chunks
        partial_chunks = [
            {
                "id": "chunk_1",
                "content": "Content 1",
                "source_url": "https://example.com/doc1",
                "document_id": "doc_1"
            },
            {
                "id": "chunk_2",
                "content": "Content 2",
                "source_url": "",  # Invalid - empty URL
                "document_id": "doc_2"
            }
        ]

        batch_result = validator.validate_batch_metadata(partial_chunks)
        assert batch_result["total_chunks"] == 2
        assert batch_result["valid_chunks"] == 1  # Only first chunk is valid
        assert batch_result["metadata_accuracy"] == 0.5

    def test_comprehensive_metadata_validation(self):
        """
        Test comprehensive metadata validation with content similarity
        """
        validator = MetadataValidatorService()

        retrieved_chunk = {
            "content": "Artificial intelligence is a wonderful field of computer science",
            "source_url": "https://wikipedia.org/ai",
            "document_id": "wiki_ai_123"
        }

        original_content = "Artificial intelligence is a wonderful field of computer science that studies intelligent agents"

        result = validator.validate_metadata_comprehensive(
            retrieved_chunk,
            original_content=original_content
        )

        assert "content_similarity" in result
        assert "metadata_consistency_score" in result
        assert result["content_similarity"] > 0.0  # Should have some similarity

    def test_source_trustworthiness_assessment(self):
        """
        Test source trustworthiness assessment
        """
        validator = MetadataValidatorService()

        # Test high trustworthiness
        high_trust_result = validator._assess_source_trustworthiness("https://en.wikipedia.org/wiki/AI")
        assert high_trust_result == "high"

        # Test medium trustworthiness
        medium_trust_result = validator._assess_source_trustworthiness("https://medium.com/story")
        assert medium_trust_result == "medium"

        # Test low trustworthiness
        low_trust_result = validator._assess_source_trustworthiness("https://example.blogspot.com")
        assert low_trust_result == "low"

    def test_content_similarity_calculation(self):
        """
        Test content similarity calculation
        """
        validator = MetadataValidatorService()

        # Test identical content
        similarity = validator._calculate_content_similarity("Hello world", "Hello world")
        assert abs(similarity - 1.0) < 0.001

        # Test completely different content
        similarity = validator._calculate_content_similarity("Hello", "Goodbye")
        assert similarity == 0.0

        # Test partially similar content
        similarity = validator._calculate_content_similarity("Hello world", "Hello there world")
        assert 0.0 < similarity < 1.0