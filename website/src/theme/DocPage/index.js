import React from 'react';
import clsx from 'clsx';
import {
  PageMetadata,
  SkipToContentFallbackId,
  ThemeClassNames,
} from '@docusaurus/theme-common';
import { useDocsSidebar } from '@docusaurus/theme-common/internal';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import DocPageLayout from '@theme/DocPage/Layout';
import DocBreadcrumbs from '@theme/DocBreadcrumbs';
import DocContent from '@theme/DocContent';
import DocItemFooter from '@theme/DocItem/Footer';
import DocItemContent from '@theme/DocItem/Content';
import DocPaginator from '@theme/DocPaginator';
import DocVersionBanner from '@theme/DocVersionBanner';
import DocVersionBadge from '@theme/DocVersionBadge';
import { useLocation } from '@docusaurus/router';
import styles from './styles.module.css';

export default function DocPage(props) {
  const { content: DocContentDefault, versionMetadata } = props;
  const { frontMatter, metadata } = DocContentDefault;
  const { title, description, wrapperClassName } = metadata;
  const { hide_title: hideTitle } = frontMatter;
  const { sidebar } = useDocsSidebar();
  const location = useLocation();

  return (
    <>
      <PageMetadata title={title} description={description} />
      <div className={clsx(ThemeClassNames.common.docPage, styles.docPage)}>
        <DocVersionBanner versionMetadata={versionMetadata} />
        <div className="container">
          <div className="row">
            <main
              className={clsx(
                'col',
                wrapperClassName,
                styles.docMainContainer,
              )}
              itemScope
              itemType="https://schema.org/Article">
              <div className="padding-horiz--md">
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
                </div>

                <div className={styles.docContent}>
                  <DocBreadcrumbs />
                  <DocVersionBadge />
                  {!hideTitle && <h1 className={styles.docTitle}>{title}</h1>}
                  <DocItemContent>
                    <DocContentDefault />
                  </DocItemContent>
                  <DocItemFooter />
                  <DocPaginator />
                </div>
              </div>
            </main>
          </div>
        </div>
      </div>
    </>
  );
}