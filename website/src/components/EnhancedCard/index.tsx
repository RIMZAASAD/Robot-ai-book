import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

interface EnhancedCardProps {
  title?: string;
  description?: string;
  icon?: React.ReactNode;
  children?: React.ReactNode;
  className?: string;
  variant?: 'default' | 'gradient' | 'glass' | 'elevated';
  hoverEffect?: boolean;
  onClick?: () => void;
}

const EnhancedCard: React.FC<EnhancedCardProps> = ({
  title,
  description,
  icon,
  children,
  className = '',
  variant = 'default',
  hoverEffect = true,
  onClick
}) => {
  const cardClasses = clsx(
    styles.enhancedCard,
    styles[`enhancedCard--${variant}`],
    {
      [styles.enhancedCardHover]: hoverEffect,
      [styles.enhancedCardClickable]: onClick
    },
    className
  );

  const renderContent = () => (
    <>
      {icon && <div className={styles.enhancedCardIcon}>{icon}</div>}
      {title && <h3 className={styles.enhancedCardTitle}>{title}</h3>}
      {description && <p className={styles.enhancedCardDescription}>{description}</p>}
      {children && <div className={styles.enhancedCardContent}>{children}</div>}
    </>
  );

  if (onClick) {
    return (
      <button className={cardClasses} onClick={onClick}>
        <div className={styles.enhancedCardInner}>
          {renderContent()}
        </div>
      </button>
    );
  }

  return (
    <div className={cardClasses}>
      <div className={styles.enhancedCardInner}>
        {renderContent()}
      </div>
    </div>
  );
};

export default EnhancedCard;