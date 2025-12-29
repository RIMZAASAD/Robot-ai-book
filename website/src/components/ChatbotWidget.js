import React, { useState } from 'react';
import './ChatbotWidget.css';

const ChatbotWidget = () => {
    const [query, setQuery] = useState('');
    const [response, setResponse] = useState('');
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState('');

    const handleSubmit = async (e) => {
        e.preventDefault();
        if (!query.trim()) return;

        setLoading(true);
        setError('');
        setResponse('');

        try {
            // Try relative endpoint first (works when site and API are served from same origin)
            const endpoints = ['/v1/chat/query', 'http://localhost:8000/v1/chat/query'];
            let res = null;

            for (const endpoint of endpoints) {
                try {
                    res = await fetch(endpoint, {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/json',
                        },
                        body: JSON.stringify({
                            query: query.trim(),
                            include_citations: true,
                            session_id: 'web_session_' + Date.now()
                        }),
                    });

                    // If we got a response (even non-OK), stop trying other endpoints
                    if (res) break;
                } catch (innerErr) {
                    // Network error (CORS, connection refused, etc.) — try next endpoint
                    console.warn(`Request to ${endpoint} failed:`, innerErr);
                    res = null;
                }
            }

            if (!res) {
                throw new Error('No backend endpoints responded');
            }

            if (!res.ok) {
                const text = await res.text().catch(() => '');
                throw new Error(`HTTP error! status: ${res.status} - ${text}`);
            }

            const data = await res.json();
            setResponse(data.response);
        } catch (err) {
            console.error('Chatbot error:', err);
            if (err.message.includes('connection refused') || err.message.includes('Failed to fetch')) {
                setError('Cannot connect to the backend server. Make sure the backend is running on port 8000.');
            } else if (err.message.includes('HTTP error!')) {
                setError(`Backend error: ${err.message}. Check if the backend service is properly configured.`);
            } else {
                setError('Failed to get response from the chatbot. Please try again.');
            }
        } finally {
            setLoading(false);
        }
    };

    return (
        <div className="chatbot-widget">
            <h3>AI Learning Assistant</h3>
            <p>Ask questions about the Physical AI & Humanoid Robotics textbook content.</p>

            <form onSubmit={handleSubmit} className="chatbot-form">
                <textarea
                    value={query}
                    onChange={(e) => setQuery(e.target.value)}
                    placeholder="Ask a question about the textbook..."
                    rows="3"
                    className="chatbot-input"
                    disabled={loading}
                />
                <button
                    type="submit"
                    className="chatbot-submit"
                    disabled={loading || !query.trim()}
                >
                    {loading ? 'Thinking...' : 'Ask'}
                </button>
            </form>

            {error && (
                <div className="chatbot-error">
                    {error}
                </div>
            )}

            {response && (
                <div className="chatbot-response">
                    <h4>Response:</h4>
                    <p>{response}</p>
                </div>
            )}
        </div>
    );
};

export default ChatbotWidget;