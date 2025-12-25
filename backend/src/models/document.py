from pydantic import BaseModel, Field
from datetime import datetime
from typing import List, Optional
from enum import Enum


class DocumentStatus(str, Enum):
    PENDING = "pending"
    PROCESSING = "processing"
    COMPLETED = "completed"
    FAILED = "failed"


class JobStatus(str, Enum):
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class Document(BaseModel):
    """
    Represents the original content source with URL, title, and metadata attributes
    """
    id: str = Field(..., description="Unique identifier for the document")
    url: str = Field(..., description="Source URL of the document")
    title: str = Field(..., description="Title of the document")
    content: str = Field(..., description="Full extracted content from the URL")
    created_at: datetime = Field(default_factory=datetime.now, description="Timestamp when document was ingested")
    updated_at: datetime = Field(default_factory=datetime.now, description="Timestamp of last update")
    status: DocumentStatus = Field(default=DocumentStatus.PENDING, description="Processing status")


class TextChunk(BaseModel):
    """
    Represents a semantically meaningful segment of extracted text with position context and content
    """
    id: str = Field(..., description="Unique identifier for the text chunk")
    document_id: str = Field(..., description="Reference to parent Document")
    content: str = Field(..., description="The actual text content of the chunk")
    chunk_index: int = Field(..., description="Position of this chunk in the original document")
    chunk_size: int = Field(..., description="Size of the chunk in characters/tokens")
    overlap_size: int = Field(default=0, description="Size of overlap with adjacent chunks")
    created_at: datetime = Field(default_factory=datetime.now, description="Timestamp when chunk was created")


class EmbeddingVector(BaseModel):
    """
    Represents the semantic vector representation of a text chunk, stored in Qdrant
    """
    id: str = Field(..., description="Unique identifier for the embedding vector")
    chunk_id: str = Field(..., description="Reference to parent TextChunk")
    vector: List[float] = Field(..., description="The actual embedding vector values")
    vector_size: int = Field(..., description="Dimension of the embedding vector")
    model_name: str = Field(..., description="Name of the model used to generate the embedding")
    created_at: datetime = Field(default_factory=datetime.now, description="Timestamp when embedding was generated")


class IngestionJob(BaseModel):
    """
    Represents a batch processing task containing multiple URLs to be processed
    """
    id: str = Field(..., description="Unique identifier for the ingestion job")
    job_name: str = Field(..., description="Name/description of the job")
    urls: List[str] = Field(..., description="List of URLs to process")
    status: JobStatus = Field(default=JobStatus.PENDING, description="Job status")
    progress: int = Field(default=0, description="Percentage of completion (0-100)")
    total_documents: int = Field(default=0, description="Total number of documents to process")
    processed_documents: int = Field(default=0, description="Number of documents processed")
    created_at: datetime = Field(default_factory=datetime.now, description="Timestamp when job was created")
    updated_at: datetime = Field(default_factory=datetime.now, description="Timestamp of last status update")
    completed_at: Optional[datetime] = Field(default=None, description="Timestamp when job was completed")