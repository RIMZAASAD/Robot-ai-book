/**
 * useChat hook for the Full-Stack Integration feature.
 *
 * This custom React hook manages the chat state, handles API communication,
 * and provides methods for submitting queries and managing the chat session.
 */

import { useState, useCallback } from 'react';
import { ChatRequest, ChatResponse, ChatState, ErrorState } from '../types/chat';
import apiClient from '../services/api-client';
import responseProcessor from '../services/response-processor';

export const useChat = () => {
  const [chatState, setChatState] = useState<ChatState>({
    currentQuery: '',
    currentResponse: null,
    isLoading: false,
    error: null,
    sessionStatus: 'idle',
    queryHistory: [],
    responseHistory: []
  });

  const submitQuery = useCallback(async (request: ChatRequest) => {
    // Update state to show loading
    setChatState(prev => ({
      ...prev,
      isLoading: true,
      error: null,
      sessionStatus: 'processing',
      currentQuery: request.query
    }));

    try {
      // Submit the query to the backend API
      const response = await apiClient.submitQuery(request);

      // Process the response for display
      const processedResponse = responseProcessor.processChatResponse(response);

      // Update the state with the response
      setChatState(prev => ({
        ...prev,
        currentResponse: processedResponse,
        isLoading: false,
        sessionStatus: 'response_ready',
        queryHistory: [...prev.queryHistory, request],
        responseHistory: [...prev.responseHistory, processedResponse]
      }));
    } catch (error: any) {
      // Handle error case
      setChatState(prev => ({
        ...prev,
        isLoading: false,
        sessionStatus: 'error',
        error: error as ErrorState
      }));
    }
  }, []);

  const clearChat = useCallback(() => {
    setChatState({
      currentQuery: '',
      currentResponse: null,
      isLoading: false,
      error: null,
      sessionStatus: 'idle',
      queryHistory: [],
      responseHistory: []
    });
  }, []);

  const updateQuery = useCallback((query: string) => {
    setChatState(prev => ({
      ...prev,
      currentQuery: query
    }));
  }, []);

  return {
    chatState,
    submitQuery,
    clearChat,
    updateQuery
  };
};