import React from 'react';
import clsx from 'clsx';
import { useDocsSidebar } from '@docusaurus/theme-common/internal';
import { useLocation } from '@docusaurus/router';
import DocPageLayout from '@theme/DocPage/Layout';
import DocPageLayoutSidebar from '@theme/DocPage/Layout/Sidebar';
import DocPageLayoutMain from '@theme/DocPage/Layout/Main';
import DocPageLayoutContent from '@theme/DocPage/Layout/Content';
import styles from './ModernBookLayout.module.css';

function useDoc() {
  const sidebar = useDocsSidebar();
  return {
    sidebar,
  };
}

export default function ModernBookLayout({ children, ...props }) {
  const { sidebar } = useDoc();
  const location = useLocation();

  const hasSidebar = sidebar && sidebar.items.length > 0;

  return (
    <DocPageLayout {...props}>
      <div className={clsx(
        'row',
        styles.modernBookLayout,
        !hasSidebar && styles.modernBookLayoutNoSidebar
      )}>
        {hasSidebar && (
          <div className={clsx('col', styles.sidebarContainer)}>
            <DocPageLayoutSidebar />
          </div>
        )}
        <div className={clsx(
          'col',
          styles.mainContent,
          !hasSidebar ? 'col--12' : 'col--8 col--offset-1'
        )}>
          <div className={styles.contentWrapper}>
            <div className={styles.contentHeader}>
              <div className={styles.breadcrumb}>
                <span className={styles.breadcrumbItem}>
                  Physical AI & Humanoid Robotics
                </span>
                <span className={styles.breadcrumbSeparator}>/</span>
                <span className={styles.breadcrumbItem}>
                  {location.pathname.includes('chapter') ? 'Chapter Content' : 'Documentation'}
                </span>
              </div>
              <div className={styles.contentActions}>
                <button className={styles.actionButton}>
                  <i className="fas fa-bookmark"></i>
                  <span>Bookmark</span>
                </button>
                <button className={styles.actionButton}>
                  <i className="fas fa-share-alt"></i>
                  <span>Share</span>
                </button>
              </div>
            </div>

            <div className={styles.contentArea}>
              <DocPageLayoutContent>
                {children}
              </DocPageLayoutContent>
            </div>

            <div className={styles.contentFooter}>
              <div className={styles.navigation}>
                <button className={styles.navButton}>
                  <i className="fas fa-arrow-left"></i>
                  Previous
                </button>
                <button className={styles.navButton}>
                  Next
                  <i className="fas fa-arrow-right"></i>
                </button>
              </div>
            </div>
          </div>
        </div>
      </div>
    </DocPageLayout>
  );
}