import React, { useEffect, useRef } from 'react';
import gsap from 'gsap';
import { ScrollTrigger } from 'gsap/ScrollTrigger';

// Register GSAP plugins
gsap.registerPlugin(ScrollTrigger);

const GSAPAnimations = () => {
  const heroRef = useRef(null);
  const cardsRef = useRef(null);
  const titleRef = useRef(null);
  const sectionRef = useRef(null);

  useEffect(() => {
    // Clean up any existing scroll triggers
    ScrollTrigger.getAll().forEach(trigger => trigger.kill());

    // Animate hero section with smooth entrance
    if (heroRef.current) {
      gsap.fromTo(heroRef.current,
        {
          opacity: 0,
          y: 60,
          scale: 0.98
        },
        {
          opacity: 1,
          y: 0,
          scale: 1,
          duration: 1.2,
          ease: 'power3.out',
          scrollTrigger: {
            trigger: heroRef.current,
            start: 'top 75%',
            end: 'bottom 25%',
            toggleActions: 'play none none reverse'
          }
        }
      );
    }

    // Animate main title with stagger effect
    if (titleRef.current) {
      const titleChars = titleRef.current.textContent.split('');
      titleRef.current.innerHTML = titleChars.map(char =>
        char === ' ' ? `<span style="margin: 0 2px">${char}</span>` : `<span>${char}</span>`
      ).join('');

      const titleSpans = titleRef.current.querySelectorAll('span');
      gsap.fromTo(titleSpans,
        {
          opacity: 0,
          y: 30,
          rotationX: -90,
          transformOrigin: 'center bottom'
        },
        {
          opacity: 1,
          y: 0,
          rotationX: 0,
          duration: 0.8,
          ease: 'power2.out',
          stagger: 0.05,
          delay: 0.3
        }
      );
    }

    // Animate content sections with cascade effect
    if (sectionRef.current) {
      const sections = sectionRef.current.querySelectorAll('.container, .section, .row');
      gsap.fromTo(sections,
        {
          opacity: 0,
          y: 40,
          skewY: 2
        },
        {
          opacity: 1,
          y: 0,
          skewY: 0,
          duration: 0.8,
          ease: 'power2.out',
          stagger: 0.15,
          scrollTrigger: {
            trigger: sectionRef.current,
            start: 'top 80%',
            toggleActions: 'play none none reverse'
          }
        }
      );
    }

    // Animate cards with stagger and enhanced effects
    if (cardsRef.current) {
      const cards = cardsRef.current.querySelectorAll('.card, .enhanced-card, .content-card, .glass-card');
      gsap.fromTo(cards,
        {
          opacity: 0,
          y: 40,
          scale: 0.95,
          rotationX: 10,
          transformOrigin: 'center bottom'
        },
        {
          opacity: 1,
          y: 0,
          scale: 1,
          rotationX: 0,
          duration: 0.8,
          ease: 'power2.out',
          stagger: 0.12,
          scrollTrigger: {
            trigger: cardsRef.current,
            start: 'top 85%',
            toggleActions: 'play none none reverse'
          }
        }
      );
    }

    // Enhanced hover animations for interactive elements
    const interactiveElements = document.querySelectorAll('.button, .card, .menu__link, .table-of-contents__link, .glass-button');
    interactiveElements.forEach((el, index) => {
      // Mouse enter animation
      el.addEventListener('mouseenter', () => {
        gsap.to(el, {
          scale: 1.03,
          y: -2,
          duration: 0.3,
          ease: 'power2.out',
          overwrite: 'auto'
        });
      });

      // Mouse leave animation
      el.addEventListener('mouseleave', () => {
        gsap.to(el, {
          scale: 1,
          y: 0,
          duration: 0.3,
          ease: 'power2.out',
          overwrite: 'auto'
        });
      });
    });

    // Floating animation for key elements
    const floatingElements = document.querySelectorAll('.hero, .card, .content-card, .glass-container');
    floatingElements.forEach(el => {
      gsap.to(el, {
        y: -8,
        duration: 3,
        ease: 'sine.inOut',
        yoyo: true,
        repeat: -1,
        repeatRefresh: true,
        overwrite: 'auto'
      });
    });

    // Cleanup function
    return () => {
      ScrollTrigger.getAll().forEach(trigger => trigger.kill());
    };
  }, []);

  return null; // This component doesn't render anything, it just handles animations
};

// Higher-order component to add GSAP animations to any component
export const withGSAPAnimation = (WrappedComponent) => {
  return (props) => {
    return (
      <>
        <WrappedComponent {...props} />
        <GSAPAnimations />
      </>
    );
  };
};

export default GSAPAnimations;