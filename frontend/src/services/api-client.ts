/**
 * API client service for the Full-Stack Integration feature.
 *
 * This module provides functions for making API calls to the backend
 * chat service from the frontend application.
 */

import { ChatRequest, ChatResponse, ErrorState } from '../types/chat';

const API_BASE_URL = process.env.NEXT_PUBLIC_API_BASE_URL || 'http://localhost:8000';

class ApiClient {
  private baseUrl: string;

  constructor() {
    this.baseUrl = API_BASE_URL;
  }

  /**
   * Submit a query to the backend RAG agent
   */
  async submitQuery(request: ChatRequest): Promise<ChatResponse> {
    const startTime = Date.now();

    try {
      const response = await fetch(`${this.baseUrl}/v1/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(request),
      });

      const duration = Date.now() - startTime;

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || `API error: ${response.status}`);
      }

      const data: ChatResponse = await response.json();

      // Add processing time to the response
      return {
        ...data,
        processing_time_ms: duration
      };
    } catch (error: any) {
      // Create error state object
      const errorState: ErrorState = {
        error_id: `error_${Date.now()}`,
        error_type: error.name || 'network',
        error_message: error.message || 'Unknown error occurred',
        user_message: error.message || 'An error occurred while processing your query',
        timestamp: new Date(),
        request_id: request.session_id,
        retry_count: 0
      };

      throw errorState;
    }
  }

  /**
   * Check the health of the backend service
   */
  async checkHealth(): Promise<any> {
    try {
      const response = await fetch(`${this.baseUrl}/v1/chat/health`);

      if (!response.ok) {
        throw new Error(`Health check failed: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Health check failed:', error);
      return { status: 'unhealthy', error: (error as Error).message };
    }
  }

  /**
   * Get chat service status
   */
  async getStatus(): Promise<any> {
    try {
      const response = await fetch(`${this.baseUrl}/v1/chat/status`);

      if (!response.ok) {
        throw new Error(`Status check failed: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Status check failed:', error);
      return { status: 'error', error: (error as Error).message };
    }
  }
}

// Create a singleton instance
const apiClient = new ApiClient();

export default apiClient;