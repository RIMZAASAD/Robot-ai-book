import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './styles.module.css';

interface HeroButton {
  text: string;
  to: string;
  variant?: 'primary' | 'secondary';
  icon?: string;
}

interface ModernHeroProps {
  title: string;
  subtitle: string;
  description: string;
  buttons?: HeroButton[];
  className?: string;
  showScrollIndicator?: boolean;
}

const ModernHero: React.FC<ModernHeroProps> = ({
  title,
  subtitle,
  description,
  buttons = [],
  className = '',
  showScrollIndicator = true
}) => {
  const robotImageUrl = useBaseUrl('/img/robot1.svg');
  
  return (
    <section className={clsx(styles.hero, className)}>
      <div className={styles.heroContainer}>
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <h1 className={styles.heroTitle}>{title}</h1>
            <p className={styles.heroSubtitle}>{subtitle}</p>
            <p className={styles.heroDescription}>{description}</p>
          </div>

          {buttons.length > 0 && (
            <div className={styles.heroButtons}>
              {buttons.map((button, index) => (
                <Link
                  key={index}
                  to={button.to}
                  className={clsx(
                    styles.heroButton,
                    styles[`heroButton${button.variant || 'primary'}`]
                  )}
                >
                  {button.icon && <span className={styles.heroButtonIcon}>{button.icon}</span>}
                  {button.text}
                </Link>
              ))}
            </div>
          )}
        </div>

        {/* Robot Image with Animation */}
        <div className={styles.robotImageContainer}>
          <img 
            src={robotImageUrl} 
            alt="Robot" 
            className={styles.robotImage}
          />
        </div>

        {showScrollIndicator && (
          <div className={styles.scrollIndicator}>
            <div className={styles.scrollIndicatorArrow}>↓</div>
          </div>
        )}
      </div>

      <div className={styles.heroBackground} />
    </section>
  );
};

export default ModernHero;