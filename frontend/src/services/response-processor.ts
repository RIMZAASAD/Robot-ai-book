/**
 * Response processor service for the Full-Stack Integration feature.
 *
 * This module provides functions for processing responses from the backend
 * and formatting them for display in the frontend application.
 */

import { ChatResponse, RetrievedChunk, Citation } from '../types/chat';

class ResponseProcessor {
  /**
   * Process the chat response from the backend and format it for display
   */
  processChatResponse(response: ChatResponse): ChatResponse {
    // Ensure all required fields are present
    const processedResponse: ChatResponse = {
      ...response,
      citations: response.citations || [],
      retrieved_chunks_count: response.retrieved_chunks_count || 0,
      processing_time_ms: response.processing_time_ms || 0,
      confidence: response.confidence || 0
    };

    return processedResponse;
  }

  /**
   * Process and format retrieved chunks for display
   */
  processRetrievedChunks(chunks: RetrievedChunk[]): RetrievedChunk[] {
    return chunks.map(chunk => ({
      ...chunk,
      content: this.sanitizeContent(chunk.content),
      similarity_score: this.normalizeSimilarityScore(chunk.similarity_score)
    }));
  }

  /**
   * Process and format citations for display
   */
  processCitations(citations: Citation[]): Citation[] {
    return citations.map(citation => ({
      ...citation,
      content_preview: citation.content_preview ? this.sanitizeContent(citation.content_preview) : undefined
    }));
  }

  /**
   * Sanitize content to prevent XSS and ensure proper display
   */
  private sanitizeContent(content: string): string {
    // Basic sanitization to prevent XSS
    // In a real application, use a proper HTML sanitization library
    return content
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/&/g, '&amp;');
  }

  /**
   * Normalize similarity score to ensure it's within the expected range
   */
  private normalizeSimilarityScore(score: number): number {
    return Math.min(Math.max(score, 0), 1); // Clamp between 0 and 1
  }

  /**
   * Format a response for display in the UI
   */
  formatResponseForDisplay(response: ChatResponse): string {
    // In a real application, this would format the response for display
    // This could include markdown processing, etc.
    return response.response;
  }

  /**
   * Extract and format metadata from the response
   */
  extractMetadata(response: ChatResponse): Record<string, any> {
    return {
      response_id: response.response_id,
      processing_time_ms: response.processing_time_ms,
      confidence: response.confidence,
      retrieved_chunks_count: response.retrieved_chunks_count,
      citations_count: response.citations.length,
      timestamp: response.timestamp
    };
  }
}

// Create a singleton instance
const responseProcessor = new ResponseProcessor();

export default responseProcessor;