import os
import asyncio
from typing import List, Dict, Any
from pydantic import BaseModel
import logging
from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException
from contextlib import asynccontextmanager
import uuid

# Import from our modular services
from src.config.settings import settings
from src.config.qdrant_config import QdrantConfig
from src.models.document import Document, TextChunk, EmbeddingVector, IngestionJob
from src.services.url_fetching import URLFetchingService
from src.services.html_extraction import HTMLExtractionService
from src.services.text_cleaning import TextCleaningService
from src.services.text_chunking import TextChunkingService
from src.services.embedding_service import EmbeddingService
from src.services.vector_storage import VectorStorageService

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Pydantic models
class IngestionRequest(BaseModel):
    urls: List[str]
    job_name: str = "default_ingestion_job"

class IngestionResponse(BaseModel):
    job_id: str
    status: str
    documents_processed: int
    vectors_stored: int

# Global variables for services
qdrant_config = None
url_fetching_service = None
html_extraction_service = None
text_cleaning_service = None
text_chunking_service = None
embedding_service = None
vector_storage_service = None

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifespan manager - initializes services
    """
    global qdrant_config, url_fetching_service, html_extraction_service, text_cleaning_service
    global text_chunking_service, embedding_service, vector_storage_service

    try:
        # Initialize Qdrant configuration
        qdrant_config = QdrantConfig()
        qdrant_client = qdrant_config.initialize_client()
        qdrant_config.initialize_collection()
        logger.info("Qdrant client and collection initialized")

        # Initialize services
        url_fetching_service = URLFetchingService()
        html_extraction_service = HTMLExtractionService()
        text_cleaning_service = TextCleaningService()
        text_chunking_service = TextChunkingService()
        embedding_service = EmbeddingService()
        vector_storage_service = VectorStorageService()

        logger.info("All services initialized successfully")

        yield

    except Exception as e:
        logger.error(f"Error during application startup: {str(e)}")
        raise

    finally:
        # Cleanup
        if hasattr(url_fetching_service, 'close'):
            await url_fetching_service.close()
        logger.info("Services shut down")

# Create FastAPI app
app = FastAPI(
    title="RAG Data Ingestion Pipeline",
    description="API for ingesting URLs, chunking text, generating embeddings, and storing in Qdrant",
    version="1.0.0",
    lifespan=lifespan
)

class IngestionPipeline:
    """Main pipeline for processing URLs through the entire RAG ingestion process"""

    @staticmethod
    async def process_urls(urls: List[str]) -> Dict[str, Any]:
        """Process a list of URLs through the entire ingestion pipeline"""
        global url_fetching_service, html_extraction_service, text_cleaning_service
        global text_chunking_service, embedding_service, vector_storage_service

        total_documents = 0
        total_vectors = 0

        all_embeddings = []
        all_texts = []
        all_metadata = []

        for url in urls:
            logger.info(f"Processing URL: {url}")

            try:
                # Step 1: Fetch content from URL
                fetched_data = await url_fetching_service.fetch_content(url)
                logger.info(f"Fetched content from {url}, length: {len(fetched_data['content'])}")

                # Step 2: Extract clean text from HTML
                extracted_data = await html_extraction_service.execute(fetched_data['content'], url)
                if not extracted_data.get("success", True):
                    logger.error(f"Failed to extract content from {url}: {extracted_data.get('error')}")
                    continue

                # Step 3: Clean the text
                cleaned_data = await text_cleaning_service.execute(extracted_data["content"])
                if not cleaned_data.get("success", True):
                    logger.error(f"Failed to clean text from {url}: {cleaned_data.get('error')}")
                    continue

                # Step 4: Chunk the text
                chunks = await text_chunking_service.execute(cleaned_data["cleaned_text"])
                logger.info(f"Created {len(chunks)} chunks from {url}")

                # Prepare chunks for embedding
                chunk_texts = [chunk["content"] for chunk in chunks]

                # Add metadata for each chunk
                for i, chunk in enumerate(chunks):
                    metadata = {
                        "title": extracted_data["title"],
                        "url": url,
                        "chunk_index": chunk["chunk_index"],
                        "document_id": f"doc_{total_documents}",
                        "created_at": "",
                        "source_type": "web_page"
                    }
                    all_metadata.append(metadata)
                    all_texts.append(chunk["content"])

                total_documents += 1

            except Exception as e:
                logger.error(f"Error processing {url}: {str(e)}")
                continue

        if all_texts:
            # Step 5: Generate embeddings
            logger.info(f"Generating embeddings for {len(all_texts)} text chunks")
            embeddings = await embedding_service.execute(all_texts)
            all_embeddings.extend(embeddings)

            # Step 6: Store embeddings in Qdrant
            logger.info(f"Storing {len(embeddings)} embeddings in Qdrant")
            stored_ids = await vector_storage_service.execute(embeddings, all_texts, all_metadata)
            total_vectors = len(stored_ids)

        return {
            "documents_processed": total_documents,
            "vectors_stored": total_vectors,
            "status": "completed"
        }

@app.post("/v1/ingest", response_model=IngestionResponse)
async def ingest_urls(request: IngestionRequest):
    """Ingest URLs through the RAG pipeline"""
    try:
        logger.info(f"Starting ingestion job: {request.job_name} with {len(request.urls)} URLs")

        # Process the URLs through the pipeline
        result = await IngestionPipeline.process_urls(request.urls)

        job_id = str(uuid.uuid4())

        response = IngestionResponse(
            job_id=job_id,
            status=result["status"],
            documents_processed=result["documents_processed"],
            vectors_stored=result["vectors_stored"]
        )

        logger.info(f"Ingestion job {job_id} completed: {response.documents_processed} docs, {response.vectors_stored} vectors")
        return response

    except Exception as e:
        logger.error(f"Error in ingestion: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


# Include validation API routes
from src.api.v1.validation import router as validation_router
app.include_router(validation_router)

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    global qdrant_config
    if qdrant_config:
        health_status = qdrant_config.health_check()
        return {
            "status": "healthy",
            "service": "RAG Data Ingestion Pipeline",
            "qdrant_status": health_status
        }
    return {"status": "healthy", "service": "RAG Data Ingestion Pipeline"}

def main():
    """Main function to run the full pipeline from command line"""
    import argparse

    parser = argparse.ArgumentParser(description="RAG Data Ingestion Pipeline")
    parser.add_argument("--urls", nargs="+", required=True, help="URLs to process")
    parser.add_argument("--job-name", default="cli_ingestion", help="Name for the ingestion job")

    args = parser.parse_args()

    # Initialize services manually since we're not using FastAPI
    global qdrant_config, url_fetching_service, html_extraction_service, text_cleaning_service
    global text_chunking_service, embedding_service, vector_storage_service

    # Load environment variables
    load_dotenv()

    try:
        # Initialize Qdrant configuration
        qdrant_config = QdrantConfig()
        qdrant_client = qdrant_config.initialize_client()
        qdrant_config.initialize_collection()
        logger.info("Qdrant client and collection initialized")

        # Initialize services
        url_fetching_service = URLFetchingService()
        html_extraction_service = HTMLExtractionService()
        text_cleaning_service = TextCleaningService()
        text_chunking_service = TextChunkingService()
        embedding_service = EmbeddingService()
        vector_storage_service = VectorStorageService()

        logger.info("All services initialized successfully")

        # Run the ingestion pipeline
        logger.info(f"Starting ingestion pipeline for URLs: {args.urls}")

        # Create a simple event loop context for async operations
        import asyncio
        result = asyncio.run(IngestionPipeline.process_urls(args.urls))

        logger.info(f"Ingestion completed: {result}")

        # Get vector count for verification
        vector_count = vector_storage_service.get_vector_count()
        verification_result = {
            "status": "verified",
            "vector_count": vector_count,
            "collection_name": settings.collection_name
        }
        logger.info(f"Verification result: {verification_result}")

        print(f"Ingestion completed successfully!")
        print(f"Documents processed: {result['documents_processed']}")
        print(f"Vectors stored: {result['vectors_stored']}")
        print(f"Qdrant verification: {verification_result}")

    except Exception as e:
        logger.error(f"Error in CLI execution: {str(e)}")
        print(f"Error: {str(e)}")
        raise

if __name__ == "__main__":
    main()