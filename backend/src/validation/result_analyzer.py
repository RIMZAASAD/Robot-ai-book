import time
from typing import List, Dict, Any
from ..models.validation_results import ValidationResult, RetrievedChunk
from ..config.validation_config import ValidationConfig
import logging


class ResultAnalyzer:
    """
    Analysis of retrieval results for validation
    """

    def __init__(self, config: ValidationConfig):
        self.config = config
        self.logger = logging.getLogger(self.__class__.__name__)

    def analyze_single_result(self, query: str, retrieved_chunks: List[Dict[str, Any]]) -> ValidationResult:
        """
        Analyze a single retrieval result for relevance and quality
        """
        start_time = time.time()

        if not retrieved_chunks:
            return ValidationResult(
                id=f"analysis_{int(time.time())}",
                query_id="unknown",
                is_relevant=False,
                relevance_score=0.0,
                metadata_valid=False,
                semantic_similarity=0.0,
                retrieval_latency=time.time() - start_time,
                deterministic=True,
                validation_details="No chunks retrieved"
            )

        # Calculate average similarity score
        total_similarity = sum(chunk.get("score", 0) for chunk in retrieved_chunks)
        avg_similarity = total_similarity / len(retrieved_chunks)

        # Check if results are relevant based on threshold
        is_relevant = avg_similarity >= self.config.validation_threshold

        # Analyze metadata validity
        metadata_valid = self._analyze_metadata_validity(retrieved_chunks)

        # Calculate determinism (for now, assume true if results exist)
        deterministic = len(retrieved_chunks) > 0

        return ValidationResult(
            id=f"analysis_{int(time.time())}",
            query_id="unknown",  # In real implementation, this would come from the query object
            is_relevant=is_relevant,
            relevance_score=avg_similarity,
            metadata_valid=metadata_valid,
            semantic_similarity=avg_similarity,
            retrieval_latency=time.time() - start_time,
            deterministic=deterministic,
            validation_details={
                "total_chunks": len(retrieved_chunks),
                "avg_similarity": avg_similarity,
                "top_chunk_score": max(chunk.get("score", 0) for chunk in retrieved_chunks),
                "metadata_analysis": self._detailed_metadata_analysis(retrieved_chunks)
            }
        )

    def analyze_batch_results(self, query_results: List[Dict[str, Any]]) -> List[ValidationResult]:
        """
        Analyze multiple retrieval results
        """
        analysis_results = []
        for result in query_results:
            query_text = result.get("query", "")
            retrieved_chunks = result.get("retrieved_chunks", [])
            analysis = self.analyze_single_result(query_text, retrieved_chunks)
            analysis_results.append(analysis)
        return analysis_results

    def _analyze_metadata_validity(self, retrieved_chunks: List[Dict[str, Any]]) -> bool:
        """
        Analyze if the metadata in retrieved chunks is valid
        """
        if not retrieved_chunks:
            return False

        # Check if all chunks have required metadata fields
        required_fields = ["source_url", "document_id"]
        valid_chunks = 0

        for chunk in retrieved_chunks:
            has_all_fields = all(field in chunk and chunk[field] for field in required_fields)
            if has_all_fields:
                valid_chunks += 1

        # Consider metadata valid if 80% of chunks have valid metadata
        validity_ratio = valid_chunks / len(retrieved_chunks)
        return validity_ratio >= 0.8

    def _detailed_metadata_analysis(self, retrieved_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Perform detailed analysis of metadata in retrieved chunks
        """
        analysis = {
            "total_chunks": len(retrieved_chunks),
            "chunks_with_source_url": 0,
            "chunks_with_document_id": 0,
            "chunks_with_content": 0,
            "unique_sources": set(),
            "metadata_completeness_score": 0.0
        }

        for chunk in retrieved_chunks:
            if chunk.get("source_url"):
                analysis["chunks_with_source_url"] += 1
                analysis["unique_sources"].add(chunk["source_url"])
            if chunk.get("document_id"):
                analysis["chunks_with_document_id"] += 1
            if chunk.get("content"):
                analysis["chunks_with_content"] += 1

        # Calculate metadata completeness score
        if analysis["total_chunks"] > 0:
            total_checks = analysis["total_chunks"] * 2  # source_url and document_id
            valid_checks = analysis["chunks_with_source_url"] + analysis["chunks_with_document_id"]
            analysis["metadata_completeness_score"] = valid_checks / total_checks

        analysis["unique_sources"] = list(analysis["unique_sources"])
        return analysis

    def calculate_validation_metrics(self, validation_results: List[ValidationResult]) -> Dict[str, Any]:
        """
        Calculate overall validation metrics from multiple results
        """
        if not validation_results:
            return {
                "total_results": 0,
                "relevant_results": 0,
                "precision": 0.0,
                "avg_relevance_score": 0.0,
                "avg_metadata_validity": 0.0,
                "avg_semantic_similarity": 0.0,
                "avg_retrieval_latency": 0.0,
                "determinism_rate": 0.0
            }

        total_results = len(validation_results)
        relevant_results = sum(1 for result in validation_results if result.is_relevant)
        precision = relevant_results / total_results if total_results > 0 else 0.0

        avg_relevance_score = sum(result.relevance_score for result in validation_results) / total_results
        avg_metadata_validity = sum(result.metadata_valid for result in validation_results) / total_results
        avg_semantic_similarity = sum(result.semantic_similarity for result in validation_results) / total_results
        avg_retrieval_latency = sum(result.retrieval_latency for result in validation_results) / total_results
        determinism_rate = sum(result.deterministic for result in validation_results) / total_results

        return {
            "total_results": total_results,
            "relevant_results": relevant_results,
            "precision": precision,
            "avg_relevance_score": avg_relevance_score,
            "avg_metadata_validity": avg_metadata_validity,
            "avg_semantic_similarity": avg_semantic_similarity,
            "avg_retrieval_latency": avg_retrieval_latency,
            "determinism_rate": determinism_rate
        }

    def generate_validation_summary(self, validation_results: List[ValidationResult]) -> Dict[str, Any]:
        """
        Generate a summary of validation results
        """
        metrics = self.calculate_validation_metrics(validation_results)

        return {
            "summary": {
                "total_queries": metrics["total_results"],
                "relevant_queries": metrics["relevant_results"],
                "precision": metrics["precision"],
                "success_rate": metrics["precision"],  # For queries that returned relevant results
                "quality_metrics": {
                    "relevance_score": metrics["avg_relevance_score"],
                    "metadata_validity": metrics["avg_metadata_validity"],
                    "semantic_similarity": metrics["avg_semantic_similarity"],
                    "latency": metrics["avg_retrieval_latency"],
                    "determinism": metrics["determinism_rate"]
                }
            },
            "compliance": {
                "meets_precision_threshold": metrics["precision"] >= self.config.min_precision_score,
                "meets_latency_threshold": metrics["avg_retrieval_latency"] <= self.config.max_retrieval_latency,
                "meets_metadata_threshold": metrics["avg_metadata_validity"] >= self.config.min_metadata_accuracy
            }
        }

    async def execute(self, query_results: List[Dict[str, Any]]) -> List[ValidationResult]:
        """
        Execute the result analysis operation
        """
        return self.analyze_batch_results(query_results)