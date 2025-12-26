/**
 * ChatContainer component for the Full-Stack Integration feature.
 *
 * This component brings together all chat interface elements and connects
 * them to the backend API through the useChat hook.
 */

import React from 'react';
import { useChat } from '../hooks/useChat';
import ChatInput from './ChatInterface/ChatInput';
import ResponseDisplay from './ChatInterface/ResponseDisplay';
import ErrorDisplay from './ChatInterface/ErrorDisplay';
import { ChatRequest } from '../types/chat';

const ChatContainer: React.FC = () => {
  const { chatState, submitQuery, clearChat, updateQuery } = useChat();

  const handleQuerySubmit = (request: ChatRequest) => {
    submitQuery(request);
  };

  const handleRetry = () => {
    if (chatState.currentQuery) {
      const request: ChatRequest = {
        query: chatState.currentQuery,
        include_citations: true,
        session_id: `sess_${Date.now()}`,
      };
      submitQuery(request);
    }
  };

  const handleDismissError = () => {
    // In a real implementation, we might want to clear the error from state
    // For now, we'll just clear the chat to reset the state
    clearChat();
  };

  return (
    <div className="chat-container">
      <div className="chat-header">
        <h1>AI Textbook Assistant</h1>
        <button onClick={clearChat} className="clear-chat-button">
          Clear Chat
        </button>
      </div>

      <div className="chat-messages">
        {chatState.error && (
          <ErrorDisplay
            error={chatState.error}
            onRetry={handleRetry}
            onDismiss={handleDismissError}
          />
        )}
        <ResponseDisplay
          response={chatState.currentResponse}
          isLoading={chatState.isLoading}
        />
      </div>

      <div className="chat-input-area">
        <ChatInput
          onQuerySubmit={handleQuerySubmit}
          isLoading={chatState.isLoading}
        />
      </div>

      <style jsx>{`
        .chat-container {
          display: flex;
          flex-direction: column;
          height: 100vh;
          max-width: 800px;
          margin: 0 auto;
          border: 1px solid #e0e0e0;
          border-radius: 8px;
          overflow: hidden;
          box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
        }

        .chat-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          padding: 1rem;
          background-color: #f8f9fa;
          border-bottom: 1px solid #e0e0e0;
        }

        .chat-header h1 {
          margin: 0;
          font-size: 1.5rem;
          color: #333;
        }

        .clear-chat-button {
          padding: 0.5rem 1rem;
          background-color: #dc3545;
          color: white;
          border: none;
          border-radius: 4px;
          cursor: pointer;
          font-size: 0.9rem;
        }

        .clear-chat-button:hover {
          background-color: #c82333;
        }

        .chat-messages {
          flex: 1;
          overflow-y: auto;
          padding: 1rem;
          background-color: #fff;
        }

        .chat-input-area {
          border-top: 1px solid #e0e0e0;
          background-color: #fff;
        }
      `}</style>
    </div>
  );
};

export default ChatContainer;