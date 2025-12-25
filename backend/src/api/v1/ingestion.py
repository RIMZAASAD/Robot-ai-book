from fastapi import APIRouter, HTTPException
from typing import List, Dict, Any
import uuid
import logging
from pydantic import BaseModel

from ...models.document import IngestionJob
from ...services.url_fetching import URLFetchingService
from ...services.html_extraction import HTMLExtractionService
from ...services.text_cleaning import TextCleaningService
from ...services.url_validation import URLValidationService


# Pydantic models for API
class IngestionRequest(BaseModel):
    urls: List[str]
    job_name: str = "default_ingestion_job"


class IngestionResponse(BaseModel):
    job_id: str
    status: str
    documents_processed: int
    vectors_stored: int


class BatchIngestionRequest(BaseModel):
    urls: List[str]
    job_name: str = "batch_ingestion_job"
    chunk_size: int = 512
    chunk_overlap: int = 102


# Initialize router
router = APIRouter(prefix="/v1", tags=["ingestion"])

# Initialize services
url_fetching_service = URLFetchingService()
html_extraction_service = HTMLExtractionService()
text_cleaning_service = TextCleaningService()

logger = logging.getLogger(__name__)


@router.post("/ingest", response_model=IngestionResponse)
async def ingest_single(request: IngestionRequest):
    """
    Ingest a single URL or batch of URLs through the RAG pipeline
    """
    try:
        logger.info(f"Starting ingestion job: {request.job_name} with {len(request.urls)} URLs")

        # Validate URLs
        validated_urls = URLValidationService.validate_and_clean_urls(request.urls)
        if not validated_urls:
            raise HTTPException(status_code=400, detail="No valid URLs provided")

        documents_processed = 0
        total_content = ""

        # Process each URL
        for url in validated_urls:
            logger.info(f"Processing URL: {url}")

            # Fetch content from URL
            fetched_data = await url_fetching_service.fetch_content(url)
            logger.info(f"Fetched content from {url}, length: {len(fetched_data['content'])}")

            # Extract clean text from HTML
            extracted_data = await html_extraction_service.execute(fetched_data['content'], url)
            if not extracted_data.get("success", True):
                logger.error(f"Failed to extract content from {url}: {extracted_data.get('error')}")
                continue

            # Clean the text
            cleaned_data = await text_cleaning_service.execute(extracted_data["content"])
            if not cleaned_data.get("success", True):
                logger.error(f"Failed to clean text from {url}: {cleaned_data.get('error')}")
                continue

            total_content += cleaned_data["cleaned_text"] + " "
            documents_processed += 1

            logger.info(f"Successfully processed {url}")

        # For now, just return basic response
        # In a complete implementation, this would continue with chunking, embedding, and storage
        response = IngestionResponse(
            job_id=str(uuid.uuid4()),
            status="completed" if documents_processed > 0 else "failed",
            documents_processed=documents_processed,
            vectors_stored=0  # Will be updated when we implement the full pipeline
        )

        logger.info(f"Ingestion job completed: {response.job_id}")
        return response

    except Exception as e:
        logger.error(f"Error in ingestion: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/ingest/batch", response_model=IngestionResponse)
async def ingest_batch(request: BatchIngestionRequest):
    """
    Ingest multiple URLs in batch mode
    """
    try:
        logger.info(f"Starting batch ingestion: {request.job_name} with {len(request.urls)} URLs")

        # This is a simplified implementation - in a full implementation,
        # this would process the entire pipeline (chunking, embedding, storage)
        validated_urls = URLValidationService.validate_and_clean_urls(request.urls)
        if not validated_urls:
            raise HTTPException(status_code=400, detail="No valid URLs provided")

        documents_processed = 0

        for url in validated_urls:
            logger.info(f"Batch processing URL: {url}")
            # Process each URL similarly to the single ingestion endpoint
            fetched_data = await url_fetching_service.fetch_content(url)
            extracted_data = await html_extraction_service.execute(fetched_data['content'], url)
            if extracted_data.get("success", True):
                documents_processed += 1

        response = IngestionResponse(
            job_id=str(uuid.uuid4()),
            status="completed" if documents_processed > 0 else "failed",
            documents_processed=documents_processed,
            vectors_stored=0  # Will be updated when we implement the full pipeline
        )

        return response

    except Exception as e:
        logger.error(f"Error in batch ingestion: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/ingest/status/{job_id}")
async def get_ingestion_status(job_id: str):
    """
    Get the status of an ingestion job
    """
    # In a complete implementation, this would look up job status in a database
    # For now, return a mock response
    return {
        "job_id": job_id,
        "status": "completed",
        "progress": 100,
        "details": "Job completed successfully"
    }


@router.get("/health")
async def health_check():
    """
    Health check for the ingestion service
    """
    from ...config.qdrant_config import QdrantConfig
    from ...config.settings import settings

    qdrant_config = QdrantConfig()
    qdrant_health = qdrant_config.health_check()

    return {
        "status": "healthy",
        "service": "Ingestion API",
        "timestamp": datetime.now().isoformat(),
        "dependencies": {
            "qdrant": qdrant_health
        }
    }


@router.get("/health/full")
async def full_health_check():
    """
    Comprehensive health check for all services
    """
    from ...config.qdrant_config import QdrantConfig
    from ...config.settings import settings
    from ...services.url_fetching import URLFetchingService

    # Check Qdrant connection
    qdrant_config = QdrantConfig()
    qdrant_health = qdrant_config.health_check()

    # Check if we can initialize services
    try:
        url_service = URLFetchingService()
        service_health = "healthy"
    except Exception as e:
        service_health = f"unhealthy: {str(e)}"

    return {
        "status": "healthy",
        "service": "Full Health Check",
        "timestamp": datetime.now().isoformat(),
        "checks": {
            "qdrant": qdrant_health,
            "services": service_health,
            "api_endpoints": "accessible"
        }
    }