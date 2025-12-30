import React from 'react';

const Robot3DViewer = ({ modelUrl = "https://sketchfab.com/3d-models/kitchen-robot-mia-971b17ac6f7048899d68cc4fb00be7fa", title = "3D Robot Model" }) => {
  return (
    <div className="robot-3d-container" style={{
      position: 'relative',
      width: '100%',
      height: '400px',
      margin: '2rem 0',
      borderRadius: '12px',
      overflow: 'hidden',
      boxShadow: '0 10px 30px rgba(0, 0, 0, 0.3)',
      background: 'linear-gradient(135deg, #1e1e24 0%, #25252d 100%)',
      border: '1px solid rgba(126, 87, 194, 0.2)'
    }}>
      <div style={{
        position: 'absolute',
        top: '0',
        left: '0',
        width: '100%',
        height: '100%',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        color: '#c1c1c4',
        fontSize: '1.1rem',
        textAlign: 'center',
        zIndex: 1
      }}>
        <div>
          <div style={{ marginBottom: '1rem', fontSize: '2rem' }}>🤖</div>
          <div>Interactive 3D Robot Model</div>
          <div style={{ fontSize: '0.9rem', opacity: 0.7, marginTop: '0.5rem' }}>
            Loading 3D visualization...
          </div>
        </div>
      </div>

      <iframe
        title={title}
        frameBorder="0"
        allowFullScreen
        allow="autoplay; fullscreen; xr-spatial-tracking; accelerometer; gyroscope"
        src={`https://sketchfab.com/models/${modelUrl.split('/').pop()}/embed?autostart=1&autoplay=1&ui_animations=1&ui_infos=1&ui_inspector=1&ui_stop=1&ui_theme=dark&ui_watermark=0&ui_watermark_link=0&ui_ar=1`}
        style={{
          width: '100%',
          height: '100%',
          border: 'none',
          zIndex: 2
        }}
      ></iframe>

      <style jsx>{`
        .robot-3d-container::before {
          content: '';
          position: absolute;
          top: -2px;
          left: -2px;
          right: -2px;
          bottom: -2px;
          background: linear-gradient(45deg, #7e57c2, #a594f9, #7e57c2, #a594f9);
          background-size: 400% 400%;
          border-radius: 14px;
          z-index: 0;
          animation: gradientFlow 4s ease-in-out infinite;
        }

        @keyframes gradientFlow {
          0% { background-position: 0% 50%; }
          50% { background-position: 100% 50%; }
          100% { background-position: 0% 50%; }
        }
      `}</style>
    </div>
  );
};

export default Robot3DViewer;