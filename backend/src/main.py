from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import logging

# Import API routers
from .api.v1 import chat, ingest
from .services.postgres_service import postgres_service

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifespan manager - runs startup and shutdown events
    """
    logger.info("Starting RAG Chatbot API...")
    try:
        # Initialize PostgreSQL service
        await postgres_service.initialize()
        logger.info("PostgreSQL service initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize PostgreSQL service: {e}")
        raise

    yield

    # Add any shutdown logic here
    try:
        await postgres_service.close()
        logger.info("PostgreSQL service closed successfully")
    except Exception as e:
        logger.error(f"Error closing PostgreSQL service: {e}")

    logger.info("Shutting down RAG Chatbot API...")

# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for the Integrated RAG Chatbot for Physical AI & Humanoid Robotics Textbook",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, restrict this to your frontend domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(chat.router, prefix="/v1", tags=["chat"])
app.include_router(ingest.router, prefix="/v1", tags=["ingest"])

# Health check endpoint
@app.get("/")
async def root():
    return {"message": "RAG Chatbot API is running!"}

@app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "RAG Chatbot API"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)