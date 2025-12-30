import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

interface ContentCardProps {
  title: string;
  description: string;
  to?: string;
  href?: string;
  icon?: string;
  level?: 'beginner' | 'intermediate' | 'advanced';
  duration?: string;
  tags?: string[];
  className?: string;
  onClick?: () => void;
}

const ContentCard: React.FC<ContentCardProps> = ({
  title,
  description,
  to,
  href,
  icon = '📚',
  level,
  duration,
  tags = [],
  className = '',
  onClick
}) => {
  const cardClasses = clsx(
    styles.contentCard,
    {
      [styles.contentCardBeginner]: level === 'beginner',
      [styles.contentCardIntermediate]: level === 'intermediate',
      [styles.contentCardAdvanced]: level === 'advanced',
      [styles.contentCardClickable]: to || href || onClick
    },
    className
  );

  const renderContent = () => (
    <>
      <div className={styles.contentCardHeader}>
        <div className={styles.contentCardIcon}>{icon}</div>
        <div className={styles.contentCardMeta}>
          {level && (
            <span className={clsx(styles.contentCardLevel, styles[`contentCardLevel${level}`])}>
              {level}
            </span>
          )}
          {duration && (
            <span className={styles.contentCardDuration}>
              {duration}
            </span>
          )}
        </div>
      </div>

      <div className={styles.contentCardTitle}>{title}</div>
      <div className={styles.contentCardDescription}>{description}</div>

      {tags.length > 0 && (
        <div className={styles.contentCardTags}>
          {tags.map((tag, index) => (
            <span key={index} className={styles.contentCardTag}>
              {tag}
            </span>
          ))}
        </div>
      )}

      <div className={styles.contentCardFooter}>
        <span className={styles.contentCardAction}>
          {to || href ? 'Read More →' : 'Learn More →'}
        </span>
      </div>
    </>
  );

  if (to) {
    return (
      <Link to={to} className={cardClasses}>
        {renderContent()}
      </Link>
    );
  }

  if (href) {
    return (
      <a href={href} className={cardClasses}>
        {renderContent()}
      </a>
    );
  }

  if (onClick) {
    return (
      <button className={cardClasses} onClick={onClick}>
        {renderContent()}
      </button>
    );
  }

  return (
    <div className={cardClasses}>
      {renderContent()}
    </div>
  );
};

export default ContentCard;