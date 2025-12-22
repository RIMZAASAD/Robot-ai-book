import React from 'react';
import clsx from 'clsx';
import { useLocation } from '@docusaurus/router';
import { useKeyboardNavigation } from '@docusaurus/theme-common/internal';
import useBaseUrl from '@docusaurus/useBaseUrl';
import ChatbotWidget from '../components/ChatbotWidget';
import '../components/ChatbotWidget.css'; // Import chatbot styles globally
import LayoutProvider from '@theme/Layout/Provider';
import Navbar from '@theme/Navbar';
import Footer from '@theme/Footer';
import ErrorPageContent from '@theme/ErrorPageContent';
import ErrorBoundary from '@docusaurus/ErrorBoundary'; // <-- FIX: import ErrorBoundary from Docusaurus

function Layout(props) {
  const { children, wrapperClassName, pageClassName } = props;
  const { pathname } = useLocation();

  useKeyboardNavigation();

  return (
    <LayoutProvider>
      <div className={clsx('main-wrapper', wrapperClassName)}>
        <Navbar />
        <main className={clsx(pageClassName)} id="main">
          {children}
        </main>
        <Footer />
      </div>
      {/* Add the chatbot widget to all pages */}
      <ChatbotWidget />
    </LayoutProvider>
  );
}

// ✅ FIX: Use imported ErrorBoundary, not React.ErrorBoundary
function LayoutWithErrorBoundary(props) {
  return (
    <ErrorBoundary

    fallback={(errorProps) => <ErrorPageContent {...errorProps} />}
    >
      <Layout {...props} />
    </ErrorBoundary>
  );
}

export default LayoutWithErrorBoundary;
