import pytest
from src.services.metadata_validator import MetadataValidatorService


class TestMetadataValidatorService:
    """
    Unit tests for the MetadataValidatorService class
    """

    def test_initialization(self):
        """
        Test that the metadata validator initializes correctly
        """
        validator = MetadataValidatorService()
        assert validator.logger is not None

    def test_validate_metadata_complete(self):
        """
        Test validating metadata with all required fields present
        """
        validator = MetadataValidatorService()

        retrieved_chunk = {
            "source_url": "https://example.com/doc1",
            "document_id": "doc_123",
            "content": "Sample content",
            "other_field": "value"
        }

        result = validator.validate_metadata(retrieved_chunk)

        assert result["metadata_valid"] is True
        assert len(result["missing_fields"]) == 0
        assert len(result["incorrect_fields"]) == 0

    def test_validate_metadata_missing_fields(self):
        """
        Test validating metadata with missing required fields
        """
        validator = MetadataValidatorService()

        retrieved_chunk = {
            "source_url": "https://example.com/doc1",
            # Missing document_id
            "content": "Sample content"
        }

        result = validator.validate_metadata(retrieved_chunk)

        assert result["metadata_valid"] is False
        assert "document_id" in result["missing_fields"]

    def test_validate_metadata_invalid_url(self):
        """
        Test validating metadata with invalid URL
        """
        validator = MetadataValidatorService()

        retrieved_chunk = {
            "source_url": "not_a_valid_url",
            "document_id": "doc_123",
            "content": "Sample content"
        }

        result = validator.validate_metadata(retrieved_chunk)

        assert result["metadata_valid"] is False
        assert "source_url" in result["incorrect_fields"]

    def test_validate_metadata_empty_fields(self):
        """
        Test validating metadata with empty required fields
        """
        validator = MetadataValidatorService()

        retrieved_chunk = {
            "source_url": "",  # Empty URL
            "document_id": "doc_123",
            "content": "Sample content"
        }

        result = validator.validate_metadata(retrieved_chunk)

        assert result["metadata_valid"] is False
        assert "source_url" in result["missing_fields"]

    def test_validate_metadata_with_original_comparison(self):
        """
        Test validating metadata by comparing with original values
        """
        validator = MetadataValidatorService()

        retrieved_chunk = {
            "source_url": "https://example.com/doc1",
            "document_id": "doc_123",
            "content": "Sample content"
        }

        original_metadata = {
            "source_url": "https://example.com/doc1",
            "document_id": "doc_123"
        }

        result = validator.validate_metadata(retrieved_chunk, original_metadata)

        assert result["metadata_valid"] is True
        assert len(result["missing_fields"]) == 0
        assert len(result["incorrect_fields"]) == 0

    def test_validate_metadata_with_incorrect_original_comparison(self):
        """
        Test validating metadata by comparing with incorrect original values
        """
        validator = MetadataValidatorService()

        retrieved_chunk = {
            "source_url": "https://example.com/doc1",
            "document_id": "doc_123",
            "content": "Sample content"
        }

        original_metadata = {
            "source_url": "https://different.com/doc1",  # Different URL
            "document_id": "doc_123"
        }

        result = validator.validate_metadata(retrieved_chunk, original_metadata)

        assert result["metadata_valid"] is False
        assert len(result["incorrect_fields"]) > 0
        assert "source_url" in result["incorrect_fields"][0]

    def test_validate_batch_metadata_all_valid(self):
        """
        Test validating batch metadata where all are valid
        """
        validator = MetadataValidatorService()

        retrieved_chunks = [
            {
                "source_url": "https://example.com/doc1",
                "document_id": "doc_123",
                "content": "Sample content 1"
            },
            {
                "source_url": "https://example.com/doc2",
                "document_id": "doc_456",
                "content": "Sample content 2"
            }
        ]

        result = validator.validate_batch_metadata(retrieved_chunks)

        assert result["total_chunks"] == 2
        assert result["valid_chunks"] == 2
        assert result["metadata_accuracy"] == 1.0

    def test_validate_batch_metadata_some_invalid(self):
        """
        Test validating batch metadata where some are invalid
        """
        validator = MetadataValidatorService()

        retrieved_chunks = [
            {
                "source_url": "https://example.com/doc1",  # Valid
                "document_id": "doc_123",  # Valid
                "content": "Sample content 1"
            },
            {
                "source_url": "",  # Invalid - empty
                "document_id": "doc_456",  # Valid
                "content": "Sample content 2"
            }
        ]

        result = validator.validate_batch_metadata(retrieved_chunks)

        assert result["total_chunks"] == 2
        assert result["valid_chunks"] == 1  # Only first chunk is valid
        assert result["metadata_accuracy"] == 0.5  # 1 out of 2 is valid

    def test_is_valid_url_positive_cases(self):
        """
        Test URL validation with valid URLs
        """
        validator = MetadataValidatorService()

        valid_urls = [
            "https://example.com",
            "http://example.com",
            "https://subdomain.example.com",
            "http://localhost",
            "https://192.168.1.1",
            "https://example.com:8080/path"
        ]

        for url in valid_urls:
            assert validator._is_valid_url(url), f"URL {url} should be valid"

    def test_is_valid_url_negative_cases(self):
        """
        Test URL validation with invalid URLs
        """
        validator = MetadataValidatorService()

        invalid_urls = [
            "not_a_url",
            "ftp://example.com",  # Unsupported protocol
            "https://",  # Incomplete
            "",  # Empty
            "example.com",  # Missing protocol
            "https://",  # Just protocol
        ]

        for url in invalid_urls:
            assert not validator._is_valid_url(url), f"URL {url} should be invalid"