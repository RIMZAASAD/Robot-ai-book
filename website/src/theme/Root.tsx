import React from 'react';
import OriginalRoot from '@theme-original/Root';
import FloatingChatWidget from '../components/FloatingChatWidget';
import { useLocation } from '@docusaurus/router';

export default function Root(props: any) {
  const location = useLocation();

  // Render the floating widget on docs/book pages and on the site root (home page)
  const showWidget = typeof location?.pathname === 'string' && (location.pathname.startsWith('/docs') || location.pathname === '/');

  return (
    <>
      <OriginalRoot {...props} />
      {showWidget && <FloatingChatWidget />}
    </>
  );
}
