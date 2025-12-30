import React, { useEffect, useRef } from 'react';

const AIBackground = () => {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    const ctx = canvas.getContext('2d');

    // Set canvas size to match window
    const resizeCanvas = () => {
      canvas.width = window.innerWidth;
      canvas.height = window.innerHeight;
    };

    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);

    // AI/Humanoid particles configuration with reference design colors
    const particles = [];
    const particleCount = 80; // Increased for richer effect

    // Create particles representing AI concepts
    for (let i = 0; i < particleCount; i++) {
      particles.push({
        x: Math.random() * canvas.width,
        y: Math.random() * canvas.height,
        vx: (Math.random() - 0.5) * 0.8, // Slightly faster movement
        vy: (Math.random() - 0.5) * 0.8,
        radius: Math.random() * 3 + 1, // Slightly larger
        opacity: Math.random() * 0.4 + 0.1,
        type: Math.floor(Math.random() * 4), // 0: neuron, 1: circuit, 2: humanoid, 3: data flow
        pulse: Math.random() * Math.PI * 2, // For pulsing effect
        pulseSpeed: Math.random() * 0.02 + 0.01
      });
    }

    // Animation loop
    const animate = () => {
      // Clear canvas with reference design background color
      ctx.fillStyle = 'rgba(13, 15, 20, 0.1)'; // Deep obsidian with transparency
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      // Draw and update particles
      particles.forEach((particle, index) => {
        // Update position
        particle.x += particle.vx;
        particle.y += particle.vy;

        // Update pulse for glowing effect
        particle.pulse += particle.pulseSpeed;

        // Wrap around edges
        if (particle.x < 0) particle.x = canvas.width;
        if (particle.x > canvas.width) particle.x = 0;
        if (particle.y < 0) particle.y = canvas.height;
        if (particle.y > canvas.height) particle.y = 0;

        // Draw particle based on type with reference design colors
        ctx.save();
        ctx.globalAlpha = particle.opacity + Math.sin(particle.pulse) * 0.1; // Pulsing effect

        switch(particle.type) {
          case 0: // Neuron-like with electric cyan
            // Draw main body
            ctx.beginPath();
            ctx.arc(particle.x, particle.y, particle.radius, 0, Math.PI * 2);
            const neuronGradient = ctx.createRadialGradient(
              particle.x, particle.y, 0,
              particle.x, particle.y, particle.radius
            );
            neuronGradient.addColorStop(0, `rgba(0, 242, 255, ${0.6 + Math.sin(particle.pulse) * 0.2})`);
            neuronGradient.addColorStop(1, `rgba(0, 242, 255, 0)`);
            ctx.fillStyle = neuronGradient;
            ctx.fill();

            // Draw connections
            particles.slice(index + 1).forEach(otherParticle => {
              const dx = particle.x - otherParticle.x;
              const dy = particle.y - otherParticle.y;
              const distance = Math.sqrt(dx * dx + dy * dy);

              if (distance < 120 && otherParticle.type !== 2) { // Don't connect to humanoid
                ctx.beginPath();
                ctx.moveTo(particle.x, particle.y);
                ctx.lineTo(otherParticle.x, otherParticle.y);
                ctx.strokeStyle = `rgba(0, 242, 255, ${0.15 * (1 - distance / 120)})`;
                ctx.lineWidth = 0.8;
                ctx.stroke();
              }
            });
            break;

          case 1: // Circuit-like with vibrant violet
            ctx.beginPath();
            ctx.rect(particle.x - particle.radius, particle.y - particle.radius,
                     particle.radius * 2, particle.radius * 2);
            const circuitGradient = ctx.createRadialGradient(
              particle.x, particle.y, 0,
              particle.x, particle.y, particle.radius * 2
            );
            circuitGradient.addColorStop(0, `rgba(138, 43, 226, ${0.6 + Math.sin(particle.pulse) * 0.2})`);
            circuitGradient.addColorStop(1, `rgba(138, 43, 226, 0)`);
            ctx.fillStyle = circuitGradient;
            ctx.fill();
            break;

          case 2: // Humanoid-like with mixed colors
            ctx.beginPath();
            ctx.arc(particle.x, particle.y, particle.radius, 0, Math.PI * 2);
            const humanoidGradient = ctx.createRadialGradient(
              particle.x, particle.y, 0,
              particle.x, particle.y, particle.radius
            );
            humanoidGradient.addColorStop(0, `rgba(138, 43, 226, ${0.5 + Math.sin(particle.pulse) * 0.2})`);
            humanoidGradient.addColorStop(1, `rgba(0, 242, 255, 0)`);
            ctx.fillStyle = humanoidGradient;
            ctx.fill();
            break;

          case 3: // Data flow particles
            ctx.beginPath();
            ctx.arc(particle.x, particle.y, particle.radius * 0.7, 0, Math.PI * 2);
            const dataGradient = ctx.createRadialGradient(
              particle.x, particle.y, 0,
              particle.x, particle.y, particle.radius * 0.7
            );
            dataGradient.addColorStop(0, `rgba(0, 242, 255, ${0.8 + Math.sin(particle.pulse) * 0.2})`);
            dataGradient.addColorStop(1, `rgba(0, 242, 255, 0)`);
            ctx.fillStyle = dataGradient;
            ctx.fill();

            // Draw trailing effect
            for (let i = 1; i <= 3; i++) {
              ctx.beginPath();
              ctx.arc(particle.x - particle.vx * i * 2, particle.y - particle.vy * i * 2,
                      particle.radius * (0.7 - i * 0.15), 0, Math.PI * 2);
              ctx.fillStyle = `rgba(0, 242, 255, ${(0.3 - i * 0.1) * (0.8 + Math.sin(particle.pulse) * 0.2)})`;
              ctx.fill();
            }
            break;
        }

        ctx.restore();
      });

      requestAnimationFrame(animate);
    };

    animate();

    // Cleanup
    return () => {
      window.removeEventListener('resize', resizeCanvas);
    };
  }, []);

  return (
    <canvas
      ref={canvasRef}
      style={{
        position: 'fixed',
        top: 0,
        left: 0,
        width: '100%',
        height: '100%',
        zIndex: -3,
        pointerEvents: 'none'
      }}
    />
  );
};

export default AIBackground;