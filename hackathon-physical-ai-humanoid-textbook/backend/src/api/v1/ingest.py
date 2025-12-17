from fastapi import APIRouter, HTTPException
import logging
from ...models.chat import IngestRequest, IngestResponse
from ...services.embedding_service import embedding_service
from ...services.qdrant_service import qdrant_service
from ...services.content_ingestion import content_ingestion_service
from ...services.postgres_service import postgres_service

logger = logging.getLogger(__name__)
router = APIRouter()

@router.post("/ingest", response_model=IngestResponse)
async def ingest_endpoint(request: IngestRequest):
    """
    Ingest endpoint to process and store textbook content with embeddings
    """
    try:
        # Chunk the content if it's large (this is a simplified approach)
        # In a real implementation, we'd want more sophisticated chunking
        content = request.content
        chunks = []
        chunk_size = 1000  # characters per chunk
        overlap = 100  # character overlap

        if len(content) <= chunk_size:
            # Content is small enough to be one chunk
            chunks = [content]
        else:
            # Split content into overlapping chunks
            start = 0
            while start < len(content):
                end = start + chunk_size
                chunk = content[start:end]

                # Add overlap if not at the end
                if end < len(content) and overlap > 0:
                    overlap_end = min(end + overlap, len(content))
                    chunk = content[start:overlap_end]

                chunks.append(chunk)
                start = end  # Move past the original chunk size

        # Generate embeddings for all chunks
        embeddings = await embedding_service.generate_embeddings(chunks)

        # Prepare metadata for each chunk
        metadata_list = []
        document_ids = []

        for i, chunk in enumerate(chunks):
            # Create a unique content ID for each chunk
            content_id = f"{request.source_file.replace('/', '_').replace('.', '_')}_{i}"

            metadata = {
                "title": f"{request.title} - Part {i+1}",
                "chapter": request.chapter,
                "section": request.section,
                "source_file": request.source_file,
                "created_at": "2025-12-15"  # In a real implementation, use actual timestamp
            }
            metadata_list.append(metadata)

            # Store content metadata in PostgreSQL
            await postgres_service.store_content_metadata({
                "content_id": content_id,
                "title": metadata["title"],
                "chapter": metadata["chapter"],
                "section": metadata["section"],
                "source_file": metadata["source_file"],
                "embedding_dimension": len(embeddings[i]) if embeddings else 1536
            })

        # Store embeddings in Qdrant
        document_ids = await qdrant_service.store_embeddings(
            texts=chunks,
            embeddings=embeddings,
            metadata_list=metadata_list
        )

        return IngestResponse(
            success=True,
            document_id=document_ids[0] if document_ids else "",
            chunks_processed=len(chunks)
        )

    except Exception as e:
        logger.error(f"Error in ingest endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.post("/ingest-textbook")
async def ingest_textbook_endpoint():
    """
    Ingest the entire textbook content from markdown files
    """
    try:
        # Get all textbook content
        all_chunks = await content_ingestion_service.ingest_textbook_content()

        if not all_chunks:
            return {"message": "No content found to ingest", "success": False}

        # Extract just the text content for embedding
        texts = [chunk['content'] for chunk in all_chunks]

        # Generate embeddings for all chunks
        embeddings = await embedding_service.generate_embeddings(texts)

        # Prepare metadata for storage
        metadata_list = []
        for i, chunk in enumerate(all_chunks):
            # Create a unique content ID for each chunk
            content_id = f"{chunk['source_file'].replace('/', '_').replace('.', '_')}_{i}"

            metadata = {
                "title": chunk['title'],
                "chapter": chunk['chapter'],
                "section": chunk['section'],
                "source_file": chunk['source_file'],
                "created_at": chunk['created_at']
            }
            metadata_list.append(metadata)

            # Store content metadata in PostgreSQL
            await postgres_service.store_content_metadata({
                "content_id": content_id,
                "title": metadata["title"],
                "chapter": metadata["chapter"],
                "section": metadata["section"],
                "source_file": metadata["source_file"],
                "embedding_dimension": len(embeddings[i]) if i < len(embeddings) else 1536
            })

        # Store embeddings in Qdrant
        document_ids = await qdrant_service.store_embeddings(
            texts=texts,
            embeddings=embeddings,
            metadata_list=metadata_list
        )

        return {
            "success": True,
            "documents_processed": len(all_chunks),
            "chunks_stored": len(document_ids)
        }

    except Exception as e:
        logger.error(f"Error in ingest textbook endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")