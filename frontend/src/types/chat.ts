/**
 * Chat-related type definitions for the Full-Stack Integration feature.
 *
 * This module contains TypeScript interfaces for chat entities
 * used in the frontend application.
 */

export interface UserQuery {
  query_id: string;
  query_text: string;
  timestamp: Date;
  session_id: string;
  user_id?: string;
}

export interface Citation {
  source_url: string;
  document_id: string;
  similarity_score: number;
  chunk_index?: number;
  content_preview?: string;
}

export interface RetrievedChunk {
  chunk_id: string;
  content: string;
  similarity_score: number;
  source_url: string;
  document_id: string;
  chunk_index: number;
  metadata: Record<string, any>;
}

export interface AgentResponse {
  response_id: string;
  content: string;
  source_citations: Citation[];
  confidence_score: number;
  timestamp: Date;
  query_id: string;
  processing_time_ms: number;
}

export interface ChatRequest {
  query: string;
  include_citations: boolean;
  session_id: string;
}

export interface ChatResponse {
  response_id: string;
  response: string;
  citations: Citation[];
  retrieved_chunks_count: number;
  processing_time_ms: number;
  confidence: number;
  session_id: string;
  timestamp: string;
}

export interface APICommunication {
  request_id: string;
  endpoint: string;
  method: string;
  request_payload: any;
  response_payload: any;
  status_code: number;
  timestamp: Date;
  duration_ms: number;
}

export interface ErrorState {
  error_id: string;
  error_type: 'network' | 'validation' | 'server' | 'timeout' | 'unknown';
  error_message: string;
  user_message: string;
  timestamp: Date;
  request_id?: string;
  retry_count: number;
}

export interface ChatState {
  currentQuery: string;
  responses: ChatResponse[];
  isLoading: boolean;
  error: ErrorState | null;
  session_id: string;
  queryHistory: UserQuery[];
}

// Status types
export type QueryStatus = 'DRAFT' | 'SUBMITTED' | 'PROCESSING' | 'COMPLETED' | 'FAILED';
export type APIStatus = 'INITIATED' | 'SENT' | 'RECEIVING' | 'COMPLETED' | 'FAILED';
export type SessionStatus = 'IDLE' | 'AWAITING_RESPONSE' | 'DISPLAYING_RESPONSE' | 'ERROR';