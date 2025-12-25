from typing import Dict, List, Any, Optional
import logging
from ..models.validation_results import RetrievedChunk, ValidationResult


class MetadataValidatorService:
    """
    Service for validating metadata preservation in retrieved chunks
    """

    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)

    def validate_metadata(self, retrieved_chunk: Dict[str, Any], original_metadata: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Validate that metadata in retrieved chunks matches expected values
        """
        try:
            validation_result = {
                "metadata_valid": True,
                "missing_fields": [],
                "incorrect_fields": [],
                "validation_details": {}
            }

            # Check required metadata fields
            required_fields = ["source_url", "document_id"]
            for field in required_fields:
                if field not in retrieved_chunk or not retrieved_chunk[field]:
                    validation_result["metadata_valid"] = False
                    validation_result["missing_fields"].append(field)

            # Validate specific fields
            if "source_url" in retrieved_chunk and retrieved_chunk["source_url"]:
                if not self._is_valid_url(retrieved_chunk["source_url"]):
                    validation_result["metadata_valid"] = False
                    validation_result["incorrect_fields"].append("source_url")

            if "document_id" in retrieved_chunk and retrieved_chunk["document_id"]:
                if not retrieved_chunk["document_id"].strip():
                    validation_result["metadata_valid"] = False
                    validation_result["incorrect_fields"].append("document_id")

            # If original metadata is provided, compare with retrieved metadata
            if original_metadata:
                for key, expected_value in original_metadata.items():
                    if key in retrieved_chunk:
                        actual_value = retrieved_chunk[key]
                        if actual_value != expected_value:
                            validation_result["metadata_valid"] = False
                            validation_result["incorrect_fields"].append(f"{key} (expected: {expected_value}, got: {actual_value})")
                    else:
                        validation_result["metadata_valid"] = False
                        validation_result["missing_fields"].append(key)

            validation_result["validation_details"]["retrieved_metadata"] = retrieved_chunk
            validation_result["validation_details"]["required_fields_present"] = len(required_fields) - len(validation_result["missing_fields"])

            self.logger.info(f"Metadata validation completed: {validation_result['metadata_valid']}")
            return validation_result

        except Exception as e:
            self.logger.error(f"Error validating metadata: {str(e)}")
            return {
                "metadata_valid": False,
                "missing_fields": [],
                "incorrect_fields": [],
                "validation_details": {"error": str(e)},
                "success": False
            }

    def validate_batch_metadata(self, retrieved_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Validate metadata for a batch of retrieved chunks
        """
        total_chunks = len(retrieved_chunks)
        valid_chunks = 0
        all_validation_results = []

        for chunk in retrieved_chunks:
            validation = self.validate_metadata(chunk)
            all_validation_results.append(validation)
            if validation["metadata_valid"]:
                valid_chunks += 1

        accuracy = valid_chunks / total_chunks if total_chunks > 0 else 0

        return {
            "total_chunks": total_chunks,
            "valid_chunks": valid_chunks,
            "metadata_accuracy": accuracy,
            "validation_results": all_validation_results
        }

    def _is_valid_url(self, url: str) -> bool:
        """
        Simple URL validation
        """
        import re
        url_pattern = re.compile(
            r'^https?://'  # http:// or https://
            r'(?:(?:[A-Z0-9](?:[A-Z0-9-]{0,61}[A-Z0-9])?\.)+[A-Z]{2,6}\.?|'  # domain...
            r'localhost|'  # localhost...
            r'\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})'  # ...or ip
            r'(?::\d+)?'  # optional port
            r'(?:/?|[/?]\S+)$', re.IGNORECASE)
        return url_pattern.match(url) is not None

    def validate_metadata_comprehensive(self, retrieved_chunk: Dict[str, Any], original_content: str = None, original_metadata: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Perform comprehensive metadata validation including content comparison
        """
        basic_validation = self.validate_metadata(retrieved_chunk, original_metadata)

        validation_result = {
            **basic_validation,
            "content_similarity_valid": True,
            "source_trustworthiness": "unknown",
            "metadata_consistency_score": 0.0
        }

        # If original content is provided, validate content similarity
        if original_content and retrieved_chunk.get("content"):
            content_similarity = self._calculate_content_similarity(
                original_content,
                retrieved_chunk["content"]
            )
            validation_result["content_similarity"] = content_similarity
            # Consider content similar if it matches at least 80%
            validation_result["content_similarity_valid"] = content_similarity >= 0.8

        # Assess source trustworthiness based on URL
        if retrieved_chunk.get("source_url"):
            validation_result["source_trustworthiness"] = self._assess_source_trustworthiness(
                retrieved_chunk["source_url"]
            )

        # Calculate overall consistency score
        consistency_score = self._calculate_metadata_consistency_score(validation_result)
        validation_result["metadata_consistency_score"] = consistency_score

        return validation_result

    def _calculate_content_similarity(self, original: str, retrieved: str) -> float:
        """
        Calculate similarity between original and retrieved content
        """
        if not original or not retrieved:
            return 0.0

        # Simple word overlap similarity
        orig_words = set(original.lower().split())
        ret_words = set(retrieved.lower().split())

        if not orig_words and not ret_words:
            return 1.0
        if not orig_words or not ret_words:
            return 0.0

        intersection = orig_words.intersection(ret_words)
        union = orig_words.union(ret_words)

        return len(intersection) / len(union)

    def _assess_source_trustworthiness(self, url: str) -> str:
        """
        Assess the trustworthiness of a source URL
        """
        trusted_domains = [
            "wikipedia.org", "wikimedia.org", "wikidata.org",
            "edu", "gov", "org", "com",
            "arxiv.org", "springer.com", "acm.org", "ieee.org"
        ]

        for domain in trusted_domains:
            if domain in url:
                return "high"

        # Check for common untrusted patterns
        untrusted_patterns = ["blogspot", "weebly", "wixsite"]
        for pattern in untrusted_patterns:
            if pattern in url:
                return "low"

        return "medium"

    def _calculate_metadata_consistency_score(self, validation_result: Dict[str, Any]) -> float:
        """
        Calculate an overall consistency score based on validation results
        """
        scores = []

        # Metadata validity contributes to the score
        if validation_result["metadata_valid"]:
            scores.append(1.0)
        else:
            scores.append(0.0)

        # Content similarity contributes to the score
        if validation_result.get("content_similarity_valid", True):
            scores.append(validation_result.get("content_similarity", 0.5))
        else:
            scores.append(0.0)

        # If there are scores, return the average
        if scores:
            return sum(scores) / len(scores)
        else:
            return 0.0

    async def execute(self, retrieved_chunks: List[Dict[str, Any]], original_metadata: Dict[str, Any] = None) -> Dict[str, Any]:
        """
        Execute the metadata validation operation
        """
        return self.validate_batch_metadata(retrieved_chunks)