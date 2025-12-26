/**
 * ErrorDisplay component for the Full-Stack Integration feature.
 *
 * This component displays error messages to users in a user-friendly way,
 * with options to retry or get more information about the error.
 */

import React from 'react';
import { ErrorState } from '../types/chat';

interface ErrorDisplayProps {
  error: ErrorState | null;
  onRetry?: () => void;
  onDismiss?: () => void;
}

const ErrorDisplay: React.FC<ErrorDisplayProps> = ({ error, onRetry, onDismiss }) => {
  if (!error) {
    return null;
  }

  const getErrorIcon = () => {
    switch (error.error_type) {
      case 'network':
        return '🌐';
      case 'timeout':
        return '⏱️';
      case 'validation':
        return '❌';
      case 'server':
        return '⚙️';
      default:
        return '⚠️';
    }
  };

  const getErrorMessage = () => {
    if (error.user_message) {
      return error.user_message;
    }

    switch (error.error_type) {
      case 'network':
        return 'Unable to connect to the server. Please check your internet connection.';
      case 'timeout':
        return 'The request took too long to complete. Please try again.';
      case 'validation':
        return 'The request contains invalid data. Please check your input.';
      case 'server':
        return 'The server encountered an error. Please try again later.';
      default:
        return 'An unexpected error occurred. Please try again.';
    }
  };

  return (
    <div className="error-display">
      <div className="error-content">
        <div className="error-header">
          <span className="error-icon">{getErrorIcon()}</span>
          <h3>Error</h3>
          {onDismiss && (
            <button className="dismiss-button" onClick={onDismiss} aria-label="Dismiss error">
              ✕
            </button>
          )}
        </div>

        <div className="error-message">
          <p>{getErrorMessage()}</p>
          {error.error_message && error.error_message !== getErrorMessage() && (
            <details className="error-details">
              <summary>Technical details</summary>
              <pre>{error.error_message}</pre>
            </details>
          )}
        </div>

        <div className="error-actions">
          {onRetry && (
            <button className="retry-button" onClick={onRetry}>
              Try Again
            </button>
          )}
          <button className="contact-support" onClick={() => alert('Contact support functionality would go here')}>
            Contact Support
          </button>
        </div>
      </div>

      <style jsx>{`
        .error-display {
          margin: 1rem;
          padding: 1rem;
          background-color: #fef7f7;
          border: 1px solid #f5c6cb;
          border-radius: 4px;
          color: #721c24;
        }

        .error-content {
          display: flex;
          flex-direction: column;
          gap: 0.75rem;
        }

        .error-header {
          display: flex;
          justify-content: space-between;
          align-items: center;
          font-weight: bold;
        }

        .error-icon {
          font-size: 1.2rem;
          margin-right: 0.5rem;
        }

        .error-header h3 {
          margin: 0;
          color: #721c24;
          flex: 1;
        }

        .dismiss-button {
          background: none;
          border: none;
          font-size: 1.2rem;
          cursor: pointer;
          color: #721c24;
          padding: 0;
          width: 24px;
          height: 24px;
          display: flex;
          align-items: center;
          justify-content: center;
        }

        .dismiss-button:hover {
          background-color: rgba(0, 0, 0, 0.1);
          border-radius: 50%;
        }

        .error-message p {
          margin: 0;
          line-height: 1.4;
        }

        .error-details {
          margin-top: 0.5rem;
        }

        .error-details summary {
          cursor: pointer;
          color: #d36e70;
          font-size: 0.85rem;
        }

        .error-details pre {
          margin: 0.5rem 0 0 0;
          padding: 0.5rem;
          background-color: #f8f9fa;
          border: 1px solid #e9ecef;
          border-radius: 4px;
          font-size: 0.8rem;
          overflow-x: auto;
          color: #495057;
        }

        .error-actions {
          display: flex;
          gap: 0.5rem;
          flex-wrap: wrap;
        }

        .retry-button, .contact-support {
          padding: 0.5rem 1rem;
          border: 1px solid;
          border-radius: 4px;
          cursor: pointer;
          font-size: 0.9rem;
        }

        .retry-button {
          background-color: #007acc;
          color: white;
          border-color: #007acc;
        }

        .retry-button:hover {
          background-color: #005a9e;
        }

        .contact-support {
          background-color: transparent;
          color: #721c24;
          border-color: #721c24;
        }

        .contact-support:hover {
          background-color: #f1b0b7;
        }
      `}</style>
    </div>
  );
};

export default ErrorDisplay;