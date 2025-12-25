from typing import List, Dict, Any
from datetime import datetime
from ..models.validation_results import ValidationReport, ValidationStatus
import logging


class MetadataReportGenerator:
    """
    Generate metadata validation reports
    """

    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)

    def generate_metadata_report(self, validation_results: List[Dict[str, Any]], report_name: str) -> Dict[str, Any]:
        """
        Generate a comprehensive metadata validation report
        """
        if not validation_results:
            return {
                "report_id": f"metadata_report_{int(datetime.now().timestamp())}",
                "validation_name": report_name,
                "status": ValidationStatus.COMPLETED,
                "total_validations": 0,
                "metadata_accuracy": 0.0,
                "summary": {
                    "total_chunks": 0,
                    "valid_chunks": 0,
                    "invalid_chunks": 0,
                    "accuracy_percentage": 0.0
                },
                "details": []
            }

        # Calculate metadata accuracy
        total_chunks = 0
        valid_chunks = 0
        invalid_chunks = 0
        source_types = {}
        trustworthiness_scores = {"high": 0, "medium": 0, "low": 0, "unknown": 0}

        detailed_results = []

        for result in validation_results:
            chunks = result.get("retrieved_chunks", [])
            total_chunks += len(chunks)

            for chunk in chunks:
                # Count valid/invalid chunks
                if chunk.get("metadata_valid", False):
                    valid_chunks += 1
                else:
                    invalid_chunks += 1

                # Track source types
                source_url = chunk.get("source_url", "")
                if source_url:
                    domain = self._extract_domain(source_url)
                    if domain in source_types:
                        source_types[domain] += 1
                    else:
                        source_types[domain] = 1

                # Track trustworthiness
                trust_level = chunk.get("source_trustworthiness", "unknown")
                if trust_level in trustworthiness_scores:
                    trustworthiness_scores[trust_level] += 1

                # Add to detailed results
                detailed_results.append({
                    "chunk_id": chunk.get("id"),
                    "source_url": source_url,
                    "document_id": chunk.get("document_id"),
                    "metadata_valid": chunk.get("metadata_valid", False),
                    "trustworthiness": trust_level,
                    "content_similarity": chunk.get("content_similarity", 0.0),
                    "consistency_score": chunk.get("metadata_consistency_score", 0.0)
                })

        # Calculate accuracy
        accuracy = valid_chunks / total_chunks if total_chunks > 0 else 0.0

        report = {
            "report_id": f"metadata_report_{int(datetime.now().timestamp())}",
            "validation_name": report_name,
            "status": ValidationStatus.COMPLETED,
            "generated_at": datetime.now().isoformat(),
            "total_validations": len(validation_results),
            "metadata_accuracy": accuracy,
            "summary": {
                "total_chunks": total_chunks,
                "valid_chunks": valid_chunks,
                "invalid_chunks": invalid_chunks,
                "accuracy_percentage": accuracy * 100,
                "source_distribution": source_types,
                "trustworthiness_distribution": trustworthiness_scores
            },
            "details": detailed_results
        }

        self.logger.info(f"Generated metadata report: {report['report_id']} with {total_chunks} chunks")
        return report

    def _extract_domain(self, url: str) -> str:
        """
        Extract domain from URL
        """
        try:
            # Remove protocol
            if "://" in url:
                url = url.split("://", 1)[1]
            # Get domain (first part before any slash)
            domain = url.split("/", 1)[0]
            return domain.lower()
        except:
            return "unknown"

    def generate_validation_summary(self, reports: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Generate a summary across multiple validation reports
        """
        if not reports:
            return {
                "total_reports": 0,
                "average_metadata_accuracy": 0.0,
                "total_chunks_validated": 0,
                "total_valid_chunks": 0,
                "overall_accuracy": 0.0
            }

        total_reports = len(reports)
        total_accuracy = sum(report.get("metadata_accuracy", 0) for report in reports)
        average_accuracy = total_accuracy / total_reports if total_reports > 0 else 0.0

        total_chunks = sum(report["summary"]["total_chunks"] for report in reports)
        total_valid_chunks = sum(report["summary"]["valid_chunks"] for report in reports)
        overall_accuracy = total_valid_chunks / total_chunks if total_chunks > 0 else 0.0

        return {
            "total_reports": total_reports,
            "average_metadata_accuracy": average_accuracy,
            "total_chunks_validated": total_chunks,
            "total_valid_chunks": total_valid_chunks,
            "overall_accuracy": overall_accuracy,
            "report_details": [
                {
                    "report_id": report.get("report_id"),
                    "validation_name": report.get("validation_name"),
                    "accuracy": report.get("metadata_accuracy"),
                    "chunks": report["summary"]["total_chunks"]
                } for report in reports
            ]
        }

    async def execute(self, validation_results: List[Dict[str, Any]], report_name: str) -> Dict[str, Any]:
        """
        Execute the metadata report generation
        """
        return self.generate_metadata_report(validation_results, report_name)