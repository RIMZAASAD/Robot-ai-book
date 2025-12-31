import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import clsx from 'clsx';
import ModernSidebar from '@site/src/components/ModernSidebar';
import { useDocsSidebar } from '@docusaurus/theme-common/internal';

export default function DocPageLayout(props) {
  const { children, ...layoutProps } = props;
  const sidebarData = useDocsSidebar();
  const [isSidebarOpen, setIsSidebarOpen] = useState(false);

  // Close sidebar on route change for mobile
  useEffect(() => {
    const handleResize = () => {
      if (window.innerWidth >= 768) {
        setIsSidebarOpen(true);
      } else {
        setIsSidebarOpen(false);
      }
    };

    handleResize();
    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  const toggleSidebar = () => {
    setIsSidebarOpen(!isSidebarOpen);
  };

  const closeSidebar = () => {
    setIsSidebarOpen(false);
  };

  return (
    <Layout {...layoutProps}>
      <div className="layout-container">
        {/* Hamburger Button - Only visible on mobile */}
        {sidebarData && sidebarData.items.length > 0 && (
          <button
            className="hamburger-menu-button"
            onClick={toggleSidebar}
            aria-label="Toggle sidebar"
            aria-expanded={isSidebarOpen}
          >
            <span className="hamburger-line"></span>
            <span className="hamburger-line"></span>
            <span className="hamburger-line"></span>
          </button>
        )}
        {sidebarData && sidebarData.items.length > 0 && (
          <ModernSidebar 
            items={sidebarData.items} 
            isOpen={isSidebarOpen}
            onClose={closeSidebar}
          />
        )}
        <main className={clsx('doc-main', { 'with-sidebar': sidebarData && sidebarData.items.length > 0 })}>
          {children}
        </main>
      </div>
    </Layout>
  );
}