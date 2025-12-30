import React from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import ModernSidebar from '@site/src/components/ModernSidebar';
import { useDocsSidebar } from '@docusaurus/theme-common/internal';

export default function DocPageLayout(props) {
  const { children, ...layoutProps } = props;
  const sidebarData = useDocsSidebar();

  return (
    <Layout {...layoutProps}>
      <div className="layout-container">
        {sidebarData && sidebarData.items.length > 0 && (
          <ModernSidebar items={sidebarData.items} />
        )}
        <main className={clsx('doc-main', { 'with-sidebar': sidebarData && sidebarData.items.length > 0 })}>
          {children}
        </main>
      </div>
    </Layout>
  );
}