/**
 * ResponseDisplay component for the Full-Stack Integration feature.
 *
 * This component displays the responses from the backend RAG agent,
 * including the main response content, citations, and metadata.
 */

import React from 'react';
import { ChatResponse, Citation } from '../../types/chat';

interface ResponseDisplayProps {
  response: ChatResponse | null;
  isLoading: boolean;
}

const ResponseDisplay: React.FC<ResponseDisplayProps> = ({ response, isLoading }) => {
  if (isLoading) {
    return (
      <div className="response-display-loading">
        <div className="loading-indicator">
          <div className="loading-spinner"></div>
          <p>AI is processing your query...</p>
          <div className="loading-progress">
            <div className="loading-bar"></div>
          </div>
        </div>

        <style jsx>{`
          .response-display-loading {
            padding: 2rem;
            text-align: center;
          }

          .loading-indicator {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 1rem;
          }

          .loading-spinner {
            width: 32px;
            height: 32px;
            border: 3px solid #f3f3f3;
            border-top: 3px solid #007acc;
            border-radius: 50%;
            animation: spin 1s linear infinite;
          }

          .loading-progress {
            width: 100%;
            max-width: 300px;
            height: 8px;
            background-color: #f0f0f0;
            border-radius: 4px;
            overflow: hidden;
          }

          .loading-bar {
            height: 100%;
            width: 30%;
            background: linear-gradient(90deg, #007acc, #00aaff);
            border-radius: 4px;
            animation: progress 2s ease-in-out infinite;
          }

          @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
          }

          @keyframes progress {
            0% { width: 30%; }
            50% { width: 70%; }
            100% { width: 30%; }
          }
        `}</style>
      </div>
    );
  }

  if (!response) {
    return (
      <div className="response-display-placeholder">
        <p>Ask a question to get started...</p>

        <style jsx>{`
          .response-display-placeholder {
            padding: 2rem;
            text-align: center;
            color: #666;
            font-style: italic;
          }
        `}</style>
      </div>
    );
  }

  const renderCitations = () => {
    if (!response.citations || response.citations.length === 0) {
      return null;
    }

    return (
      <div className="citations-section">
        <h3>References</h3>
        <div className="citations-list">
          {response.citations.map((citation: Citation, index: number) => (
            <div key={index} className="citation-item">
              <div className="citation-header">
                <a
                  href={citation.source_url}
                  target="_blank"
                  rel="noopener noreferrer"
                  className="citation-link"
                >
                  {citation.source_url || 'Source'}
                </a>
                <span className="similarity-score">
                  Similarity: {(citation.similarity_score * 100).toFixed(1)}%
                </span>
              </div>
              <p className="citation-preview">{citation.content_preview}</p>
            </div>
          ))}
        </div>

        <style jsx>{`
          .citations-section {
            margin-top: 1.5rem;
            padding-top: 1rem;
            border-top: 1px solid #e0e0e0;
          }

          .citations-section h3 {
            margin: 0 0 1rem 0;
            font-size: 1.1rem;
            color: #333;
          }

          .citations-list {
            display: flex;
            flex-direction: column;
            gap: 1rem;
          }

          .citation-item {
            padding: 0.75rem;
            background-color: #f9f9f9;
            border-radius: 4px;
            border-left: 3px solid #007acc;
          }

          .citation-header {
            display: flex;
            justify-content: space-between;
            align-items: flex-start;
            margin-bottom: 0.5rem;
          }

          .citation-link {
            color: #007acc;
            text-decoration: none;
            font-weight: 500;
            flex: 1;
            margin-right: 1rem;
          }

          .citation-link:hover {
            text-decoration: underline;
          }

          .similarity-score {
            font-size: 0.85rem;
            color: #666;
            white-space: nowrap;
          }

          .citation-preview {
            margin: 0;
            color: #555;
            font-size: 0.9rem;
            line-height: 1.4;
          }
        `}</style>
      </div>
    );
  };

  const renderMetadata = () => {
    return (
      <div className="metadata-section">
        <div className="metadata-grid">
          <div className="metadata-item">
            <span className="metadata-label">Confidence:</span>
            <span className="metadata-value">{(response.confidence * 100).toFixed(1)}%</span>
          </div>
          <div className="metadata-item">
            <span className="metadata-label">Chunks Retrieved:</span>
            <span className="metadata-value">{response.retrieved_chunks_count}</span>
          </div>
          <div className="metadata-item">
            <span className="metadata-label">Processing Time:</span>
            <span className="metadata-value">{response.processing_time_ms}ms</span>
          </div>
        </div>

        <style jsx>{`
          .metadata-section {
            margin-top: 1rem;
            padding: 0.75rem;
            background-color: #f5f5f5;
            border-radius: 4px;
            font-size: 0.85rem;
          }

          .metadata-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
            gap: 1rem;
          }

          .metadata-item {
            display: flex;
            justify-content: space-between;
          }

          .metadata-label {
            font-weight: 500;
            color: #555;
          }

          .metadata-value {
            font-weight: 600;
            color: #333;
          }
        `}</style>
      </div>
    );
  };

  return (
    <div className="response-display">
      <div className="response-content">
        <div
          className="response-text"
          dangerouslySetInnerHTML={{ __html: response.response }}
        />
      </div>

      {renderMetadata()}
      {renderCitations()}

      <style jsx>{`
        .response-display {
          padding: 1rem;
          max-width: 800px;
          margin: 0 auto;
          width: 100%;
        }

        .response-content {
          margin-bottom: 1rem;
        }

        .response-text {
          line-height: 1.6;
          color: #333;
          font-size: 1rem;
        }

        /* Basic markdown styling */
        .response-text :global(p) {
          margin: 0 0 1rem 0;
        }

        .response-text :global(h1),
        .response-text :global(h2),
        .response-text :global(h3) {
          margin: 1.5rem 0 0.5rem 0;
          color: #222;
        }

        .response-text :global(h1) { font-size: 1.5rem; }
        .response-text :global(h2) { font-size: 1.3rem; }
        .response-text :global(h3) { font-size: 1.1rem; }

        .response-text :global(ul),
        .response-text :global(ol) {
          margin: 0.5rem 0 1rem 0;
          padding-left: 1.5rem;
        }

        .response-text :global(li) {
          margin-bottom: 0.25rem;
        }

        .response-text :global(code) {
          background-color: #f4f4f4;
          padding: 0.2rem 0.4rem;
          border-radius: 3px;
          font-family: monospace;
        }

        .response-text :global(pre) {
          background-color: #f4f4f4;
          padding: 0.8rem;
          border-radius: 4px;
          overflow-x: auto;
          margin: 1rem 0;
        }

        .response-text :global(pre code) {
          background: none;
          padding: 0;
        }

        .response-text :global(blockquote) {
          border-left: 3px solid #007acc;
          padding-left: 1rem;
          margin: 1rem 0;
          color: #666;
          font-style: italic;
        }
      `}</style>
    </div>
  );
};

export default ResponseDisplay;