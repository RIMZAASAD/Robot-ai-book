import React, { useState, useEffect, useRef } from 'react';
import { useColorMode } from '@docusaurus/theme-common';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
// CSS is imported globally in Layout.js to avoid component import issues

const ChatbotWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [isSelectingMode, setIsSelectingMode] = useState(false);
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  const { colorMode } = useColorMode();
  const isDarkMode = colorMode === 'dark';

  // Function to get selected text
  const getSelectedText = () => {
    if (ExecutionEnvironment.canUseDOM) {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        setSelectedText(selectedText);
        setIsSelectingMode(true);
        setIsOpen(true); // Open chat when text is selected
      }
    }
  };

  // Add event listener for text selection
  useEffect(() => {
    if (ExecutionEnvironment.canUseDOM) {
      const handleMouseUp = () => {
        setTimeout(() => {
          const selectedText = window.getSelection().toString().trim();
          if (selectedText && selectedText.length > 10) { // Only if meaningful text is selected
            setSelectedText(selectedText);
          }
        }, 0);
      };

      document.addEventListener('mouseup', handleMouseUp);
      return () => {
        document.removeEventListener('mouseup', handleMouseUp);
      };
    }
  }, []);

  // Auto-scroll to bottom of messages
  useEffect(() => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages]);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { role: 'user', content: inputValue, timestamp: new Date() };
    const newMessages = [...messages, userMessage];
    setMessages(newMessages);
    setInputValue('');
    setIsLoading(true);

    try {
      // Determine which endpoint to use based on mode
      const endpoint = isSelectingMode && selectedText
        ? 'http://localhost:8000/v1/chat-selected'
        : 'http://localhost:8000/v1/chat';

      // Get current page context only in browser environment
      const pageContext = ExecutionEnvironment.canUseDOM ? window.location.pathname : '';

      const requestBody = isSelectingMode && selectedText
        ? {
          question: inputValue,
          selected_text: selectedText,
          session_id: 'docusaurus-session-' + Date.now()
        }
        : {
          question: inputValue,
          session_id: 'docusaurus-session-' + Date.now(),
          page_context: pageContext
        };

      const response = await fetch(endpoint, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const botMessage = {
        role: 'assistant',
        content: data.response,
        confidence: data.confidence_score,
        citations: data.citations || [],
        timestamp: new Date()
      };

      setMessages(prev => [...prev, botMessage]);

      // Reset selection mode after sending
      if (isSelectingMode) {
        setIsSelectingMode(false);
        setSelectedText('');
      }
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage = {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        error: true,
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => {
        if (inputRef.current) {
          inputRef.current.focus();
        }
      }, 100);
    }
  };

  const clearChat = () => {
    setMessages([]);
    setSelectedText('');
    setIsSelectingMode(false);
  };

  return (
    <div className="chatbot-widget">
      {isOpen ? (
        <div className={`chatbot-container ${isDarkMode ? 'dark' : 'light'}`}>
          <div className="chatbot-header">
            <div className="chatbot-header-left">
              <h3>📚 Textbook Assistant</h3>
              {isSelectingMode && (
                <span className="selection-indicator" title={`Using selected text: ${selectedText.substring(0, 50)}...`}>
                  📝 Selected Text Mode
                </span>
              )}
            </div>
            <div className="chatbot-header-right">
              <button
                onClick={clearChat}
                className="chatbot-clear-btn"
                title="Clear chat"
              >
                🗑️
              </button>
              <button
                onClick={toggleChat}
                className="chatbot-close-btn"
              >
                ×
              </button>
            </div>
          </div>

          <div className="chatbot-messages">
            {messages.length === 0 ? (
              <div className="chatbot-welcome">
                <p>👋 Hello! I'm your textbook assistant.</p>
                <p>{isSelectingMode
                  ? `I'll answer based on the selected text: "${selectedText.substring(0, 100)}${selectedText.length > 100 ? '...' : ''}"`
                  : 'Ask me anything about the textbook content!'}</p>
                <p className="chatbot-hint">
                  💡 Tip: Select text on the page and I'll answer based only on that text!
                </p>
              </div>
            ) : (
              messages.map((message, index) => (
                <div
                  key={index}
                  className={`chatbot-message ${message.role} ${message.error ? 'error' : ''} ${isDarkMode ? 'dark' : 'light'}`}
                >
                  <div className="message-content">
                    {message.role === 'assistant' && message.confidence !== undefined && (
                      <div className="confidence-score">
                        💯 Confidence: {(message.confidence * 100).toFixed(0)}%
                      </div>
                    )}
                    <p>{message.content}</p>
                    {message.citations && message.citations.length > 0 && (
                      <div className="citations">
                        <details>
                          <summary>Sources ({message.citations.length})</summary>
                          {message.citations.map((citation, idx) => (
                            <div key={idx} className="citation-item">
                              <strong>{citation.title}</strong>
                              <small>Chapter: {citation.chapter}</small>
                            </div>
                          ))}
                        </details>
                      </div>
                    )}
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className={`chatbot-message assistant ${isDarkMode ? 'dark' : 'light'}`}>
                <div className="typing-indicator">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chatbot-input-area">
            {isSelectingMode && (
              <div className="current-selection">
                <small>📖 Using selected text: "{selectedText.substring(0, 60)}{selectedText.length > 60 ? '...' : ''}"</small>
                <button
                  onClick={() => {
                    setIsSelectingMode(false);
                    setSelectedText('');
                  }}
                  className="remove-selection-btn"
                >
                  ✕
                </button>
              </div>
            )}
            <div className="chatbot-input-container">
              <textarea
                ref={inputRef}
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder={isSelectingMode
                  ? "Ask about the selected text..."
                  : "Ask about the textbook content..."
                }
                className={`chatbot-input ${isDarkMode ? 'dark' : 'light'}`}
                rows="1"
                disabled={isLoading}
              />
              <button
                onClick={sendMessage}
                className="chatbot-send-btn"
                disabled={isLoading || !inputValue.trim()}
              >
                {isLoading ? '⏳' : '➤'}
              </button>
            </div>
          </div>
        </div>
      ) : null}

      <button
        className={`chatbot-toggle-btn ${isOpen ? 'open' : ''} ${isDarkMode ? 'dark' : 'light'}`}
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        {isOpen ? '✕' : '🤖'}
      </button>
    </div>
  );
};

export default ChatbotWidget;