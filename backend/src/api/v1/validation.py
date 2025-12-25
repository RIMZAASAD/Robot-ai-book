from fastapi import APIRouter, HTTPException
from typing import List, Dict, Any
import uuid
import logging
from pydantic import BaseModel

from ...validation.retrieval_validator import RetrievalValidator
from ...config.validation_config import validation_config
from ...models.validation_results import ValidationReport, ValidationStatus


# Pydantic models for API
class ValidationRequest(BaseModel):
    queries: List[str]
    validation_name: str = "validation_run"
    top_k: int = 5
    similarity_threshold: float = 0.7


class ValidationResponse(BaseModel):
    report_id: str
    status: str
    total_queries: int
    message: str


class ValidationReportResponse(BaseModel):
    report_id: str
    validation_name: str
    status: ValidationStatus
    metrics: Dict[str, Any]
    results: List[Dict[str, Any]]


# Initialize router
router = APIRouter(prefix="/v1", tags=["validation"])

# Initialize validation service
validator = RetrievalValidator(validation_config)

logger = logging.getLogger(__name__)


@router.post("/validation/run", response_model=ValidationResponse)
async def run_validation(request: ValidationRequest):
    """
    Run validation on the retrieval pipeline with provided queries
    """
    try:
        logger.info(f"Starting validation run: {request.validation_name} with {len(request.queries)} queries")

        # Update config with request parameters
        validation_config.validation_top_k = request.top_k
        validation_config.validation_threshold = request.similarity_threshold

        # Run the validation
        report = await validator.run_validation(request.queries, request.validation_name)

        response = ValidationResponse(
            report_id=report.id,
            status=report.status.value,
            total_queries=report.total_queries,
            message=f"Validation completed with {report.total_queries} queries"
        )

        logger.info(f"Validation run completed: {report.id}")
        return response

    except Exception as e:
        logger.error(f"Error in validation run: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/validation/report/{report_id}", response_model=ValidationReportResponse)
async def get_validation_report(report_id: str):
    """
    Get the results of a validation run
    """
    try:
        logger.info(f"Retrieving validation report: {report_id}")

        # In a real implementation, this would fetch the report from storage
        # For now, we'll return a mock response
        # In a complete implementation, we would store reports and retrieve them
        raise HTTPException(status_code=404, detail="Report storage not implemented in this example")

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error retrieving validation report: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/validation/health")
async def validation_health():
    """
    Health check for the validation service
    """
    from ...config.qdrant_config import QdrantConfig
    from ...config.settings import settings

    qdrant_config = QdrantConfig()
    qdrant_health = qdrant_config.health_check()

    return {
        "status": "healthy",
        "service": "RAG Retrieval Validation API",
        "timestamp": "",
        "dependencies": {
            "qdrant": qdrant_health
        }
    }


@router.get("/validation/health/full")
async def full_validation_health():
    """
    Comprehensive health check for all validation services
    """
    from ...config.qdrant_config import QdrantConfig
    from ...config.settings import settings
    from ...services.embedding_service import EmbeddingService

    # Check Qdrant connection
    qdrant_config = QdrantConfig()
    qdrant_health = qdrant_config.health_check()

    # Check if we can initialize services
    try:
        embedding_service = EmbeddingService()
        service_health = "healthy"
    except Exception as e:
        service_health = f"unhealthy: {str(e)}"

    return {
        "status": "healthy",
        "service": "Full Validation Health Check",
        "timestamp": "",
        "checks": {
            "qdrant": qdrant_health,
            "embedding_service": service_health,
            "api_endpoints": "accessible"
        }
    }


# Add a simple test endpoint
@router.get("/validation/test")
async def test_validation():
    """
    Test endpoint for validation functionality
    """
    try:
        # Run a simple test with a sample query
        sample_queries = ["What is artificial intelligence?"]
        report = await validator.run_validation(sample_queries, "test_run")

        return {
            "status": "success",
            "test_queries": len(sample_queries),
            "report_id": report.id,
            "precision": report.precision_score,
            "metadata_accuracy": report.metadata_accuracy
        }
    except Exception as e:
        logger.error(f"Error in validation test: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))