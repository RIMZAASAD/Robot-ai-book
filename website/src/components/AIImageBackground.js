import React from 'react';

const AIImageBackground = () => {
  return (
    <div
      style={{
        position: 'fixed',
        top: 0,
        left: 0,
        width: '100%',
        height: '100%',
        zIndex: -4,
        pointerEvents: 'none',
        background: `linear-gradient(135deg,
          rgba(13, 15, 20, 0.9) 0%,
          rgba(22, 27, 34, 0.8) 50%,
          rgba(13, 15, 20, 0.9) 100%),
          url('https://images.unsplash.com/photo-1677442135722-5f11e06a4e6d?ixlib=rb-4.0.3&ixid=M3wxMjA3fDB8MHxwaG90by1wYWdlfHx8fGVufDB8fHx8fA%3D%3D&auto=format&fit=crop&w=2327&q=80')`,
        backgroundSize: 'cover',
        backgroundPosition: 'center',
        backgroundRepeat: 'no-repeat',
        opacity: 0.15, // Subtle background that doesn't interfere with content
      }}
    />
  );
};

export default AIImageBackground;