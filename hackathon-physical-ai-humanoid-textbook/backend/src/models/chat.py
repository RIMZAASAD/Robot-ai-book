from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime

class ChatRequest(BaseModel):
    question: str
    session_id: Optional[str] = None
    page_context: Optional[str] = None

class SelectedTextChatRequest(BaseModel):
    question: str
    selected_text: str
    session_id: Optional[str] = None

class ChatResponse(BaseModel):
    response: str
    confidence_score: float
    citations: List[dict]
    session_id: str
    response_time: float

class SelectedTextChatResponse(BaseModel):
    response: str
    confidence_score: float
    source_text_used: str
    session_id: str
    response_time: float

class IngestRequest(BaseModel):
    content: str
    title: str
    chapter: str
    section: str
    source_file: str

class IngestResponse(BaseModel):
    success: bool
    document_id: str
    chunks_processed: int