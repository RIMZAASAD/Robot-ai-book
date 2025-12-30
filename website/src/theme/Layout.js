import React from 'react';
import clsx from 'clsx';
import { useLocation } from '@docusaurus/router';
import { useKeyboardNavigation } from '@docusaurus/theme-common/internal';
import useBaseUrl from '@docusaurus/useBaseUrl';
import LayoutProvider from '@theme/Layout/Provider';
import Navbar from '@theme/Navbar';
import Footer from '@theme/Footer';
import ErrorPageContent from '@theme/ErrorPageContent';
import ErrorBoundary from '@docusaurus/ErrorBoundary';
import AIBackground from '@site/src/components/AIBackground';
import AIImageBackground from '@site/src/components/AIImageBackground';
import GSAPAnimations from '@site/src/components/GSAPAnimations';

function Layout(props) {
  const { children, wrapperClassName, pageClassName } = props;
  const { pathname } = useLocation();

  useKeyboardNavigation();

  return (
    <LayoutProvider>
      <AIImageBackground />
      <AIBackground />
      <GSAPAnimations />
      <div className={clsx('main-wrapper', wrapperClassName)}>
        <Navbar />
        <main className={clsx(pageClassName)} id="main">
          {children}
        </main>
        <Footer />
      </div>
    </LayoutProvider>
  );
}

function LayoutWithErrorBoundary(props) {
  return (
    <ErrorBoundary fallback={(errorProps) => <ErrorPageContent {...errorProps} />}>
      <Layout {...props} />
    </ErrorBoundary>
  );
}

export default LayoutWithErrorBoundary;
