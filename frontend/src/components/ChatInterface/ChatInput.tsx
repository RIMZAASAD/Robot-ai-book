/**
 * ChatInput component for the Full-Stack Integration feature.
 *
 * This component provides the interface for users to enter queries
 * and submit them to the backend RAG agent.
 */

import React, { useState, useRef, useEffect } from 'react';
import { ChatRequest } from '../../types/chat';

interface ChatInputProps {
  onQuerySubmit: (request: ChatRequest) => void;
  isLoading: boolean;
}

const ChatInput: React.FC<ChatInputProps> = ({ onQuerySubmit, isLoading }) => {
  const [query, setQuery] = useState<string>('');
  const [isFocused, setIsFocused] = useState<boolean>(false);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();

    if (query.trim() && !isLoading) {
      const chatRequest: ChatRequest = {
        query: query.trim(),
        include_citations: true,
        session_id: `sess_${Date.now()}`,
      };

      onQuerySubmit(chatRequest);
      setQuery('');
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    // Submit on Enter (without Shift for new line)
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      if (!isLoading) {
        handleSubmit(e as any);
      }
    }
  };

  // Focus the input when component mounts
  useEffect(() => {
    if (inputRef.current) {
      inputRef.current.focus();
    }
  }, []);

  return (
    <div className="chat-input-container">
      <form onSubmit={handleSubmit} className="chat-input-form">
        <div className={`chat-input-wrapper ${isFocused ? 'focused' : ''}`}>
          <textarea
            ref={inputRef}
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            onKeyDown={handleKeyDown}
            onFocus={() => setIsFocused(true)}
            onBlur={() => setIsFocused(false)}
            placeholder="Ask a question about AI concepts..."
            disabled={isLoading}
            className="chat-input-textarea"
            rows={1}
            aria-label="Enter your question"
          />
          <button
            type="submit"
            disabled={isLoading || !query.trim()}
            className="chat-submit-button"
            aria-label="Submit question"
          >
            {isLoading ? 'Sending...' : 'Send'}
          </button>
        </div>
        <div className="chat-input-hints">
          <small>Press Enter to submit, Shift+Enter for new line</small>
        </div>
      </form>

      <style jsx>{`
        .chat-input-container {
          padding: 1rem;
          border-top: 1px solid #e0e0e0;
          background-color: #fff;
        }

        .chat-input-form {
          display: flex;
          flex-direction: column;
          gap: 0.5rem;
        }

        .chat-input-wrapper {
          display: flex;
          align-items: flex-end;
          gap: 0.5rem;
          padding: 0.5rem;
          border: 2px solid #e0e0e0;
          border-radius: 8px;
          transition: border-color 0.2s ease;
        }

        .chat-input-wrapper.focused {
          border-color: #007acc;
        }

        .chat-input-textarea {
          flex: 1;
          border: none;
          outline: none;
          resize: none;
          font-size: 1rem;
          font-family: inherit;
          padding: 0.25rem;
          max-height: 120px;
        }

        .chat-submit-button {
          padding: 0.5rem 1rem;
          background-color: #007acc;
          color: white;
          border: none;
          border-radius: 4px;
          cursor: pointer;
          transition: background-color 0.2s ease;
        }

        .chat-submit-button:disabled {
          background-color: #cccccc;
          cursor: not-allowed;
        }

        .chat-submit-button:not(:disabled):hover {
          background-color: #005a9e;
        }

        .chat-input-hints {
          text-align: right;
          color: #666;
          font-size: 0.75rem;
        }
      `}</style>
    </div>
  );
};

export default ChatInput;