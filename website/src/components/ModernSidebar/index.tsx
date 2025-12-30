import React, { useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';
import styles from './styles.module.css';

interface SidebarItem {
  type: 'category' | 'doc';
  label: string;
  id?: string;
  to?: string;
  items?: SidebarItem[];
  collapsible?: boolean;
  collapsed?: boolean;
}

interface ModernSidebarProps {
  items: SidebarItem[];
  className?: string;
}

const ModernSidebar: React.FC<ModernSidebarProps> = ({ items, className = '' }) => {
  const [expandedCategories, setExpandedCategories] = useState<Set<string>>(new Set());
  const location = useLocation();

  const toggleCategory = (label: string) => {
    const newExpanded = new Set(expandedCategories);
    if (newExpanded.has(label)) {
      newExpanded.delete(label);
    } else {
      newExpanded.add(label);
    }
    setExpandedCategories(newExpanded);
  };

  const isActive = (to?: string, id?: string) => {
    if (to && location.pathname === to) {
      return true;
    }
    if (id) {
      const path = `/docs/${id.replace(/\.md$/, '')}`;
      return location.pathname === path;
    }
    return false;
  };

  const renderSidebarItem = (item: SidebarItem, depth = 0) => {
    const isExpanded = expandedCategories.has(item.label);
    const hasChildren = item.items && item.items.length > 0;

    if (item.type === 'category') {
      return (
        <div key={item.label} className={clsx(styles.sidebarItem, styles.sidebarCategory)}>
          <button
            className={clsx(styles.sidebarCategoryButton, {
              [styles.sidebarCategoryButtonActive]: hasActiveChild(item)
            })}
            onClick={() => toggleCategory(item.label)}
            style={{ paddingLeft: `${depth * 1.5 + 1}rem` }}
          >
            <span className={styles.sidebarCategoryIcon}>
              {hasChildren && (isExpanded ? '▼' : '▶')}
            </span>
            <span className={styles.sidebarCategoryLabel}>{item.label}</span>
          </button>
          {hasChildren && isExpanded && (
            <div className={styles.sidebarCategoryItems}>
              {item.items?.map((subItem) => renderSidebarItem(subItem, depth + 1))}
            </div>
          )}
        </div>
      );
    }

    return (
      <div key={item.label} className={styles.sidebarItem}>
        <Link
          to={item.to || (item.id ? `/docs/${item.id}` : '#')}
          className={clsx(styles.sidebarLink, {
            [styles.sidebarLinkActive]: isActive(item.to, item.id)
          })}
          style={{ paddingLeft: `${depth * 1.5 + 1}rem` }}
        >
          {item.label}
        </Link>
      </div>
    );
  };

  const hasActiveChild = (category: SidebarItem): boolean => {
    if (!category.items) return false;

    for (const item of category.items) {
      if (item.type === 'doc' && isActive(item.to, item.id)) {
        return true;
      }
      if (item.type === 'category' && hasActiveChild(item)) {
        return true;
      }
    }
    return false;
  };

  return (
    <aside className={clsx(styles.sidebar, className)}>
      <nav className={styles.sidebarNav}>
        {items.map((item) => renderSidebarItem(item))}
      </nav>
    </aside>
  );
};

export default ModernSidebar;