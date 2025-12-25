import asyncio
import time
from typing import List, Dict, Any, Optional
from ..models.validation_results import Query, ValidationResult, ValidationReport, ValidationStatus
from ..services.vector_search import VectorSearchService
from ..services.metadata_validator import MetadataValidatorService
from ..config.validation_config import ValidationConfig
import logging


class RetrievalValidator:
    """
    Main validation service for RAG retrieval pipeline
    """

    def __init__(self, config: ValidationConfig):
        self.config = config
        self.logger = logging.getLogger(self.__class__.__name__)
        self.vector_search_service = VectorSearchService()
        self.metadata_validator = MetadataValidatorService()
        self.validation_results = []

    async def validate_single_query(self, query_text: str) -> Dict[str, Any]:
        """
        Validate a single query against the retrieval pipeline
        """
        start_time = time.time()

        try:
            # Process the query through the pipeline
            query_result = await self._process_query(query_text)

            # Validate the results
            validation_result = self._validate_results(query_result)

            # Calculate processing time
            processing_time = time.time() - start_time

            return {
                "query": query_text,
                "query_id": query_result.get("query_id"),
                "retrieved_chunks": query_result.get("retrieved_chunks", []),
                "validation_result": validation_result,
                "processing_time": processing_time
            }

        except Exception as e:
            self.logger.error(f"Error validating query '{query_text}': {str(e)}")
            return {
                "query": query_text,
                "error": str(e),
                "validation_result": {
                    "is_relevant": False,
                    "relevance_score": 0.0,
                    "metadata_valid": False,
                    "semantic_similarity": 0.0,
                    "retrieval_latency": time.time() - start_time,
                    "deterministic": False,
                    "validation_details": f"Error during validation: {str(e)}"
                }
            }

    async def validate_batch_queries(self, queries: List[str]) -> List[Dict[str, Any]]:
        """
        Validate multiple queries in batch mode
        """
        results = []
        for query in queries:
            result = await self.validate_single_query(query)
            results.append(result)
        return results

    async def _process_query(self, query_text: str) -> Dict[str, Any]:
        """
        Process a query through the retrieval pipeline
        """
        start_time = time.time()

        # In a real implementation, this would:
        # 1. Generate embedding for the query using Cohere
        # 2. Search for similar chunks in Qdrant
        # 3. Return the results
        from ..services.embedding_service import EmbeddingService
        from ..config.settings import settings

        # Initialize embedding service
        embedding_service = EmbeddingService(
            api_key=settings.cohere_api_key,
            model_name=settings.cohere_model
        )

        # Generate query embedding
        query_embedding = await embedding_service.execute([query_text])
        query_embedding = query_embedding[0]  # Get the first (and only) embedding

        # Search for similar chunks
        similar_chunks = self.vector_search_service.search_similar_chunks(
            query_embedding=query_embedding,
            top_k=self.config.validation_top_k,
            threshold=self.config.validation_threshold
        )

        processing_time = time.time() - start_time

        return {
            "query_id": f"query_{int(time.time())}",
            "retrieved_chunks": similar_chunks,
            "processing_time": processing_time
        }

    def _validate_results(self, query_result: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate the retrieval results for relevance and metadata
        """
        retrieved_chunks = query_result.get("retrieved_chunks", [])
        processing_time = query_result.get("processing_time", 0)

        if not retrieved_chunks:
            return {
                "is_relevant": False,
                "relevance_score": 0.0,
                "metadata_valid": False,
                "semantic_similarity": 0.0,
                "retrieval_latency": processing_time,
                "deterministic": True,  # No results is deterministic
                "validation_details": "No chunks retrieved for query"
            }

        # Validate metadata for all chunks
        metadata_validation = self.metadata_validator.validate_batch_metadata(retrieved_chunks)

        # Calculate average relevance score
        total_similarity = sum(chunk.get("score", 0) for chunk in retrieved_chunks)
        avg_similarity = total_similarity / len(retrieved_chunks) if retrieved_chunks else 0

        # Determine if results are relevant based on threshold
        is_relevant = avg_similarity >= self.config.validation_threshold

        return {
            "is_relevant": is_relevant,
            "relevance_score": avg_similarity,
            "metadata_valid": metadata_validation["metadata_accuracy"] >= 0.9,  # 90% threshold
            "semantic_similarity": avg_similarity,
            "retrieval_latency": processing_time,
            "deterministic": True,  # For now, assume deterministic
            "validation_details": {
                "total_chunks": len(retrieved_chunks),
                "metadata_accuracy": metadata_validation["metadata_accuracy"],
                "avg_similarity": avg_similarity
            }
        }

    def _validate_relevance(self, similarity_score: float, threshold: float) -> bool:
        """
        Validate if a similarity score meets the relevance threshold
        """
        return similarity_score >= threshold

    def _calculate_precision(self, validation_results: List[Dict[str, Any]]) -> float:
        """
        Calculate precision based on validation results
        """
        if not validation_results:
            return 0.0

        relevant_count = sum(1 for result in validation_results if result.get("is_relevant", False))
        return relevant_count / len(validation_results)

    async def generate_validation_report(self, validation_results: List[Dict[str, Any]], report_name: str) -> ValidationReport:
        """
        Generate a comprehensive validation report
        """
        if not validation_results:
            return ValidationReport(
                id=f"report_{int(time.time())}",
                validation_name=report_name,
                total_queries=0,
                successful_retrievals=0,
                precision_score=0.0,
                average_latency=0.0,
                metadata_accuracy=0.0,
                determinism_score=0.0,
                status=ValidationStatus.COMPLETED
            )

        # Calculate metrics
        total_queries = len(validation_results)
        successful_retrievals = sum(1 for result in validation_results if result.get("retrieved_chunks"))
        precision_score = self._calculate_precision([r.get("validation_result", {}) for r in validation_results])
        average_latency = sum(r.get("processing_time", 0) for r in validation_results) / total_queries

        # Calculate metadata accuracy across all results
        metadata_valid_count = 0
        total_chunks = 0
        for result in validation_results:
            chunks = result.get("retrieved_chunks", [])
            total_chunks += len(chunks)
            for chunk in chunks:
                metadata_validation = self.metadata_validator.validate_metadata(chunk)
                if metadata_validation["metadata_valid"]:
                    metadata_valid_count += 1

        metadata_accuracy = metadata_valid_count / total_chunks if total_chunks > 0 else 0.0

        # For determinism, we'll assume all results are deterministic for now
        determinism_score = 1.0

        return ValidationReport(
            id=f"report_{int(time.time())}",
            validation_name=report_name,
            total_queries=total_queries,
            successful_retrievals=successful_retrievals,
            precision_score=precision_score,
            average_latency=average_latency,
            metadata_accuracy=metadata_accuracy,
            determinism_score=determinism_score,
            status=ValidationStatus.COMPLETED,
            results=validation_results
        )

    async def run_validation(self, queries: List[str], report_name: str = "validation_run") -> ValidationReport:
        """
        Run a complete validation process with multiple queries
        """
        self.logger.info(f"Starting validation run: {report_name} with {len(queries)} queries")

        # Validate all queries
        validation_results = await self.validate_batch_queries(queries)

        # Generate report
        report = await self.generate_validation_report(validation_results, report_name)

        self.logger.info(f"Validation run completed: {report_name}, precision: {report.precision_score:.2f}")

        return report