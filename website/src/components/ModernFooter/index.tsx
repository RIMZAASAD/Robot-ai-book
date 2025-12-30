import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

interface FooterLink {
  title: string;
  items: {
    label: string;
    to?: string;
    href?: string;
  }[];
}

interface ModernFooterProps {
  links: FooterLink[];
  copyright: string;
  className?: string;
}

const ModernFooter: React.FC<ModernFooterProps> = ({ links, copyright, className = '' }) => {
  return (
    <footer className={clsx(styles.footer, className)}>
      <div className={styles.footerContainer}>
        <div className={styles.footerContent}>
          {links.map((linkGroup, index) => (
            <div key={index} className={styles.footerColumn}>
              <h3 className={styles.footerTitle}>{linkGroup.title}</h3>
              <ul className={styles.footerList}>
                {linkGroup.items.map((item, itemIndex) => (
                  <li key={itemIndex} className={styles.footerListItem}>
                    {item.to ? (
                      <Link to={item.to} className={styles.footerLink}>
                        {item.label}
                      </Link>
                    ) : (
                      <a href={item.href} className={styles.footerLink} target="_blank" rel="noopener noreferrer">
                        {item.label}
                      </a>
                    )}
                  </li>
                ))}
              </ul>
            </div>
          ))}
        </div>

        <div className={styles.footerBottom}>
          <div className={styles.footerCopyright}>
            {copyright}
          </div>
          <div className={styles.footerSocial}>
            <a href="https://github.com/RIMZAASAD" className={styles.footerSocialLink} target="_blank" rel="noopener noreferrer">
              GitHub
            </a>
            <a href="https://www.linkedin.com/in/rimza-asad-206b332b8/" className={styles.footerSocialLink} target="_blank" rel="noopener noreferrer">
              LinkedIn
            </a>
            <a href="https://www.instagram.com/rimza218/" className={styles.footerSocialLink} target="_blank" rel="noopener noreferrer">
              Instagram
            </a>
          </div>
        </div>
      </div>
    </footer>
  );
};

export default ModernFooter;