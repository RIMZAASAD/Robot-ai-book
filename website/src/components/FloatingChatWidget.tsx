import React, { useEffect, useState, useRef } from 'react';

interface Message {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
}

const FloatingChatWidget: React.FC = () => {
  const [open, setOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      text: 'Hello! I\'m your AI assistant for Physical AI & Humanoid Robotics. Ask me anything about the textbook content.',
      sender: 'bot',
      timestamp: new Date(),
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [serviceHealthy, setServiceHealthy] = useState<boolean | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (open) document.body.style.overflow = 'hidden';
    else document.body.style.overflow = '';
    return () => { document.body.style.overflow = ''; };
  }, [open]);

  // Ping backend health endpoint periodically to show service status
  useEffect(() => {
    let mounted = true;
    const checkHealth = async () => {
      try {
        const res = await fetch('/v1/chat/health');
        if (!mounted) return;
        setServiceHealthy(res.ok);
      } catch (err) {
        if (!mounted) return;
        setServiceHealthy(false);
      }
    };

    checkHealth();
    const interval = setInterval(checkHealth, 30000);
    return () => { mounted = false; clearInterval(interval); };
  }, []);

  useEffect(() => {
    // Scroll to bottom when messages change
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date(),
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Try relative endpoint first (works when site and API are served from same origin)
      const endpoints = ['/v1/chat/query', 'http://localhost:8000/v1/chat/query'];
      let response = null;
      let error = null;

      for (const endpoint of endpoints) {
        try {
          const res = await fetch(endpoint, {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
            },
            body: JSON.stringify({
              query: inputValue.trim(),
              include_citations: true,
              session_id: 'web_session_' + Date.now()
            }),
          });

          if (res.ok) {
            response = await res.json();
            break;
          } else {
            error = `HTTP error! status: ${res.status}`;
          }
        } catch (err) {
          error = err;
          console.warn(`Request to ${endpoint} failed:`, err);
        }
      }

      if (response) {
        const botMessage: Message = {
          id: Date.now().toString(),
          text: response.response || 'Sorry, I couldn\'t process that request.',
          sender: 'bot',
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, botMessage]);
      } else {
        const errorMessage: Message = {
          id: Date.now().toString(),
          text: 'Sorry, I\'m having trouble connecting to the AI service. Please try again later.',
          sender: 'bot',
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (err) {
      console.error('Chatbot error:', err);
      const errorMessage: Message = {
        id: Date.now().toString(),
        text: 'An error occurred while processing your request. Please try again.',
        sender: 'bot',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      sendMessage();
    }
  };

  return (
    <>
      <div className={`fcw-panel ${open ? 'open' : ''}`} role="dialog" aria-hidden={!open}>
        <div className="fcw-panel-inner">
          <div className="fcw-panel-header">
            <div className="fcw-title">AI Textbook Assistant</div>
            <div className="fcw-header-buttons">
              <button
                className="fcw-clear-history"
                aria-label="Clear chat history"
                onClick={() => setMessages([
                  {
                    id: '1',
                    text: 'Hello! I\'m your AI assistant for Physical AI & Humanoid Robotics. Ask me anything about the textbook content.',
                    sender: 'bot',
                    timestamp: new Date(),
                  }
                ])}
              >
                🗑️
              </button>
              <button className="fcw-close" aria-label="Close chat" onClick={() => setOpen(false)}>✕</button>
            </div>
          </div>
          <div className="fcw-panel-body">
            <div className="fcw-chat-container">
              <div className="fcw-chat-messages">
                {messages.map((message) => (
                  <div
                    key={message.id}
                    className={`fcw-message ${message.sender === 'user' ? 'fcw-user-message' : 'fcw-bot-message'}`}
                  >
                    <div className={`fcw-message-avatar ${message.sender === 'user' ? 'fcw-user-avatar' : 'fcw-bot-avatar'}`}>
                      {message.sender === 'user' ? '👤' : '🤖'}
                    </div>
                    <div className="fcw-message-content">
                      <div className={`fcw-message-label ${message.sender === 'user' ? 'fcw-user-label' : 'fcw-bot-label'}`}>
                        {message.sender === 'user' ? 'User' : 'Bot'}
                      </div>
                      <div className={`fcw-message-bubble ${message.sender === 'user' ? 'fcw-user-bubble' : 'fcw-bot-bubble'}`}>
                        {message.text}
                      </div>
                      <div className={`fcw-message-timestamp ${message.sender === 'user' ? 'fcw-user-timestamp' : 'fcw-bot-timestamp'}`}>
                        {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                      </div>
                    </div>
                  </div>
                ))}
                {isLoading && (
                  <div className="fcw-message fcw-bot-message">
                    <div className="fcw-message-avatar fcw-bot-avatar">🤖</div>
                    <div className="fcw-message-content">
                      <div className="fcw-message-label fcw-bot-label">Bot</div>
                      <div className="fcw-message-bubble fcw-bot-bubble">
                        <div className="fcw-typing-indicator">
                          <div className="fcw-typing-dot"></div>
                          <div className="fcw-typing-dot"></div>
                          <div className="fcw-typing-dot"></div>
                        </div>
                      </div>
                    </div>
                  </div>
                )}
                <div ref={messagesEndRef} />
              </div>

              <div className="fcw-chat-input-container">
                <input
                  type="text"
                  value={inputValue}
                  onChange={(e) => setInputValue(e.target.value)}
                  onKeyPress={handleKeyPress}
                  placeholder="Ask a question..."
                  className="fcw-chat-input"
                  disabled={isLoading}
                />
                <button
                  className="fcw-send-button"
                  onClick={sendMessage}
                  disabled={isLoading || !inputValue.trim()}
                >
                  {isLoading ? 'Sending...' : 'Send'}
                </button>
              </div>
            </div>
          </div>
        </div>
      </div>

      <button
        className={`fcw-button`}
        aria-label={open ? 'Close chat' : 'Open chat'}
        onClick={() => setOpen(prev => !prev)}
        title={open ? 'Close chat' : (serviceHealthy === null ? 'Checking chat service...' : serviceHealthy ? 'Chat service: healthy' : 'Chat service: offline')}
      >
        {open ? '✕' : '💬'}
        <span className="fcw-pulse" />
        <span className={`fcw-status ${serviceHealthy ? 'healthy' : serviceHealthy === false ? 'unhealthy' : 'unknown'}`} />
      </button>

      <style>{`
        .fcw-button {
          position: fixed;
          right: 20px;
          bottom: 24px;
          width: 56px;
          height: 56px;
          border-radius: 999px;
          background: linear-gradient(135deg,#0b1220,#111827);
          color: #fff;
          display: flex;
          align-items: center;
          justify-content: center;
          box-shadow: 0 6px 18px rgba(2,6,23,0.6);
          border: 1px solid rgba(255,255,255,0.04);
          cursor: pointer;
          z-index: 9999;
          transition: transform 200ms, opacity 200ms;
          font-size: 20px;
        }
        .fcw-button:hover { transform: translateY(-3px) scale(1.02); }
        .fcw-pulse { position: absolute; width: 100%; height: 100%; border-radius: 999px; box-shadow: 0 0 0 0 rgba(99,102,241,0.35); animation: pulse 2.4s infinite; opacity: 0.8; }
        @keyframes pulse { 0% { box-shadow: 0 0 0 0 rgba(99,102,241,0.16); } 70% { box-shadow: 0 0 0 18px rgba(99,102,241,0); } 100% { box-shadow: 0 0 0 0 rgba(99,102,241,0); } }
        .fcw-button.hidden { opacity: 0; pointer-events: none; transform: scale(0.96); }

        .fcw-status { position: absolute; top: 6px; right: 6px; width: 10px; height: 10px; border-radius: 50%; border: 1px solid rgba(0,0,0,0.2); box-shadow: 0 0 6px rgba(0,0,0,0.3); }
        .fcw-status.healthy { background: #22c55e; box-shadow: 0 0 8px rgba(34,197,94,0.4); }
        .fcw-status.unhealthy { background: #ef4444; box-shadow: 0 0 8px rgba(239,68,68,0.4); }
        .fcw-status.unknown { background: #f59e0b; opacity: 0.95; }

        .fcw-panel { position: fixed; right: 20px; bottom: 92px; width: 420px; max-width: calc(100% - 40px); height: 640px; max-height: calc(100% - 48px); border-radius: 12px; background: rgba(6,6,6,0.95); box-shadow: 0 20px 50px rgba(2,6,23,0.6); opacity: 0; transform: translateX(20px) translateY(10px) scale(0.99); transition: transform 300ms cubic-bezier(.2,.9,.3,1), opacity 300ms; z-index: 9998; pointer-events: none; backdrop-filter: blur(6px); border: 1px solid rgba(255,255,255,0.04); }
        .fcw-panel.open { opacity: 1; transform: translateX(0) translateY(0) scale(1); pointer-events: auto; }
        .fcw-panel-inner { display:flex; flex-direction:column; height:100%; width:100%; overflow:hidden; border-radius:12px; }
        .fcw-panel-header { display:flex; align-items:center; justify-content:space-between; padding:12px 16px; border-bottom:1px solid rgba(255,255,255,0.03); }
        .fcw-title { font-weight:600; color:#fff; font-size:14px; }
        .fcw-header-buttons { display: flex; gap: 8px; }
        .fcw-clear-history, .fcw-close { background:transparent; border:none; color:#ddd; font-size:18px; cursor:pointer; padding:6px; border-radius:8px; }
        .fcw-clear-history:hover, .fcw-close:hover { background: rgba(255,255,255,0.02); }
        .fcw-panel-body { flex:1; min-height:0; background:transparent; padding:0; }

        .fcw-chat-container { display: flex; flex-direction: column; height: 100%; }
        .fcw-chat-messages { flex: 1; padding: 16px; overflow-y: auto; display: flex; flex-direction: column; gap: 12px; }
        .fcw-message { display: flex; align-items: flex-start; gap: 8px; max-width: 80%; margin-bottom: 8px; }
        .fcw-user-message { align-self: flex-end; flex-direction: row-reverse; }
        .fcw-bot-message { align-self: flex-start; }
        .fcw-message-avatar { width: 32px; height: 32px; border-radius: 50%; display: flex; align-items: center; justify-content: center; flex-shrink: 0; font-size: 16px; background: rgba(255,255,255,0.1); }
        .fcw-user-avatar { background: rgba(37,99,235,0.2); }
        .fcw-bot-avatar { background: rgba(255,255,255,0.1); }
        .fcw-message-content { display: flex; flex-direction: column; max-width: calc(100% - 40px); }
        .fcw-message-label { font-size: 12px; font-weight: 600; margin-bottom: 4px; color: #94a3b8; }
        .fcw-user-label { text-align: right; }
        .fcw-bot-label { text-align: left; }
        .fcw-message-bubble { padding: 10px 14px; border-radius: 18px; font-size: 14px; line-height: 1.4; max-width: 100%; }
        .fcw-user-bubble { background: #2563eb; color: white; border-bottom-right-radius: 4px; align-self: flex-end; }
        .fcw-bot-bubble { background: rgba(255,255,255,0.1); color: #fff; border-bottom-left-radius: 4px; align-self: flex-start; }
        .fcw-message-timestamp { font-size: 11px; color: #aaa; margin-top: 4px; text-align: right; }
        .fcw-user-timestamp { text-align: right; }
        .fcw-bot-timestamp { text-align: left; }

        .fcw-chat-input-container { display: flex; padding: 12px; border-top: 1px solid rgba(255,255,255,0.03); background: rgba(0,0,0,0.2); }
        .fcw-chat-input { flex: 1; padding: 10px 14px; border: 1px solid rgba(255,255,255,0.1); border-radius: 20px; background: rgba(255,255,255,0.05); color: white; font-size: 14px; outline: none; }
        .fcw-chat-input:focus { border-color: #2563eb; }
        .fcw-send-button { margin-left: 8px; padding: 10px 16px; background: #2563eb; color: white; border: none; border-radius: 20px; cursor: pointer; font-size: 14px; }
        .fcw-send-button:disabled { background: #4b5563; cursor: not-allowed; }

        .fcw-typing-indicator { display: flex; align-items: center; padding: 8px 0; }
        .fcw-typing-dot { width: 8px; height: 8px; background: #aaa; border-radius: 50%; margin: 0 2px; animation: typing 1.4s infinite ease-in-out; }
        .fcw-typing-dot:nth-child(1) { animation-delay: 0s; }
        .fcw-typing-dot:nth-child(2) { animation-delay: 0.2s; }
        .fcw-typing-dot:nth-child(3) { animation-delay: 0.4s; }
        @keyframes typing { 0%, 60%, 100% { transform: translateY(0); } 30% { transform: translateY(-5px); } }

        @media (max-width:640px) {
          .fcw-button { right:12px; bottom:16px; }
          .fcw-panel { left:12px; right:12px; bottom:16px; width:auto; height:80vh; max-height:80vh; border-radius:12px; }
        }
      `}</style>

      <style>{`
        .fcw-button {
          position: fixed;
          right: 20px;
          bottom: 24px;
          width: 56px;
          height: 56px;
          border-radius: 999px;
          background: linear-gradient(135deg,#0b1220,#111827);
          color: #fff;
          display: flex;
          align-items: center;
          justify-content: center;
          box-shadow: 0 6px 18px rgba(2,6,23,0.6);
          border: 1px solid rgba(255,255,255,0.04);
          cursor: pointer;
          z-index: 9999;
          transition: transform 200ms, opacity 200ms;
          font-size: 20px;
        }
        .fcw-button:hover { transform: translateY(-3px) scale(1.02); }
        .fcw-pulse { position: absolute; width: 100%; height: 100%; border-radius: 999px; box-shadow: 0 0 0 0 rgba(99,102,241,0.35); animation: pulse 2.4s infinite; opacity: 0.8; }
        @keyframes pulse { 0% { box-shadow: 0 0 0 0 rgba(99,102,241,0.16); } 70% { box-shadow: 0 0 0 18px rgba(99,102,241,0); } 100% { box-shadow: 0 0 0 0 rgba(99,102,241,0); } }
        .fcw-button.hidden { opacity: 0; pointer-events: none; transform: scale(0.96); }

        .fcw-panel { position: fixed; right: 20px; bottom: 92px; width: 420px; max-width: calc(100% - 40px); height: 640px; max-height: calc(100% - 48px); border-radius: 12px; background: rgba(6,6,6,0.95); box-shadow: 0 20px 50px rgba(2,6,23,0.6); opacity: 0; transform: translateX(20px) translateY(10px) scale(0.99); transition: transform 300ms cubic-bezier(.2,.9,.3,1), opacity 300ms; z-index: 9998; pointer-events: none; backdrop-filter: blur(6px); border: 1px solid rgba(255,255,255,0.04); }
        .fcw-panel.open { opacity: 1; transform: translateX(0) translateY(0) scale(1); pointer-events: auto; }
        .fcw-panel-inner { display:flex; flex-direction:column; height:100%; width:100%; overflow:hidden; border-radius:12px; }
        .fcw-panel-header { display:flex; align-items:center; justify-content:space-between; padding:12px 16px; border-bottom:1px solid rgba(255,255,255,0.03); }
        .fcw-title { font-weight:600; color:#fff; font-size:14px; }
        .fcw-header-buttons { display: flex; gap: 8px; }
        .fcw-clear-history, .fcw-close { background:transparent; border:none; color:#ddd; font-size:18px; cursor:pointer; padding:6px; border-radius:8px; }
        .fcw-clear-history:hover, .fcw-close:hover { background: rgba(255,255,255,0.02); }
        .fcw-panel-body { flex:1; min-height:0; background:transparent; padding:0; }

        .fcw-chat-container { display: flex; flex-direction: column; height: 100%; }
        .fcw-chat-messages { flex: 1; padding: 16px; overflow-y: auto; display: flex; flex-direction: column; gap: 12px; }
        .fcw-message { display: flex; align-items: flex-start; gap: 8px; max-width: 80%; margin-bottom: 8px; }
        .fcw-user-message { align-self: flex-end; flex-direction: row-reverse; }
        .fcw-bot-message { align-self: flex-start; }
        .fcw-message-avatar { width: 32px; height: 32px; border-radius: 50%; display: flex; align-items: center; justify-content: center; flex-shrink: 0; font-size: 16px; background: rgba(255,255,255,0.1); }
        .fcw-user-avatar { background: rgba(37,99,235,0.2); }
        .fcw-bot-avatar { background: rgba(255,255,255,0.1); }
        .fcw-message-content { display: flex; flex-direction: column; max-width: calc(100% - 40px); }
        .fcw-message-label { font-size: 12px; font-weight: 600; margin-bottom: 4px; color: #94a3b8; }
        .fcw-user-label { text-align: right; }
        .fcw-bot-label { text-align: left; }
        .fcw-message-bubble { padding: 10px 14px; border-radius: 18px; font-size: 14px; line-height: 1.4; max-width: 100%; }
        .fcw-user-bubble { background: #2563eb; color: white; border-bottom-right-radius: 4px; align-self: flex-end; }
        .fcw-bot-bubble { background: rgba(255,255,255,0.1); color: #fff; border-bottom-left-radius: 4px; align-self: flex-start; }
        .fcw-message-timestamp { font-size: 11px; color: #aaa; margin-top: 4px; text-align: right; }
        .fcw-user-timestamp { text-align: right; }
        .fcw-bot-timestamp { text-align: left; }

        .fcw-chat-input-container { display: flex; padding: 12px; border-top: 1px solid rgba(255,255,255,0.03); background: rgba(0,0,0,0.2); }
        .fcw-chat-input { flex: 1; padding: 10px 14px; border: 1px solid rgba(255,255,255,0.1); border-radius: 20px; background: rgba(255,255,255,0.05); color: white; font-size: 14px; outline: none; }
        .fcw-chat-input:focus { border-color: #2563eb; }
        .fcw-send-button { margin-left: 8px; padding: 10px 16px; background: #2563eb; color: white; border: none; border-radius: 20px; cursor: pointer; font-size: 14px; }
        .fcw-send-button:disabled { background: #4b5563; cursor: not-allowed; }

        .fcw-typing-indicator { display: flex; align-items: center; padding: 8px 0; }
        .fcw-typing-dot { width: 8px; height: 8px; background: #aaa; border-radius: 50%; margin: 0 2px; animation: typing 1.4s infinite ease-in-out; }
        .fcw-typing-dot:nth-child(1) { animation-delay: 0s; }
        .fcw-typing-dot:nth-child(2) { animation-delay: 0.2s; }
        .fcw-typing-dot:nth-child(3) { animation-delay: 0.4s; }
        @keyframes typing { 0%, 60%, 100% { transform: translateY(0); } 30% { transform: translateY(-5px); } }

        @media (max-width:640px) {
          .fcw-button { right:12px; bottom:16px; }
          .fcw-panel { left:12px; right:12px; bottom:16px; width:auto; height:80vh; max-height:80vh; border-radius:12px; }
        }
      `}</style>
    </>
  );
};

export default FloatingChatWidget;
