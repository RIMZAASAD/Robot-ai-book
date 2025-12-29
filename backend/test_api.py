import asyncio
from fastapi import FastAPI
from contextlib import asynccontextmanager
from fastapi.middleware.cors import CORSMiddleware
import logging

# Import from our modular services
from src.config.settings import settings

# Global variables for services
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
    global url_fetching_service, html_extraction_service, text_cleaning_service
    global text_chunking_service, embedding_service, vector_storage_service

    try:
        # Initialize services
        from src.services.url_fetching import URLFetchingService
        from src.services.html_extraction import HTMLExtractionService
        from src.services.text_cleaning import TextCleaningService
        from src.services.text_chunking import TextChunkingService
        from src.services.embedding_service import EmbeddingService
        from src.services.vector_storage import VectorStorageService
        from src.config.qdrant_config import QdrantConfig

        # Initialize Qdrant configuration
        qdrant_config = QdrantConfig()
        qdrant_client = qdrant_config.initialize_client()
        qdrant_config.initialize_collection()
        logging.info("Qdrant client and collection initialized")

        # Initialize services
        url_fetching_service = URLFetchingService()
        html_extraction_service = HTMLExtractionService()
        text_cleaning_service = TextCleaningService()
        text_chunking_service = TextChunkingService()
        embedding_service = EmbeddingService()
        vector_storage_service = VectorStorageService()

        logging.info("All services initialized successfully")

        yield

    except Exception as e:
        logging.error(f"Error during application startup: {str(e)}")
        raise

    finally:
        # Cleanup
        if hasattr(url_fetching_service, 'close'):
            await url_fetching_service.close()
        logging.info("Services shut down")

# Create FastAPI app
app = FastAPI(
    title="RAG Data Ingestion Pipeline - Test",
    description="API for testing basic endpoints",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware to allow frontend communication
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "service": "Test RAG Data Ingestion Pipeline"
    }

# Include chat API routes (without agent initialization for now)
from src.api.v1.chat import router as chat_router
app.include_router(chat_router)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)