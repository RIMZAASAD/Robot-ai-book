import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';
import styles from './styles.module.css';

interface NavItem {
  label: string;
  to?: string;
  href?: string;
  type?: 'link' | 'dropdown';
  items?: NavItem[];
}

interface ModernNavbarProps {
  items: NavItem[];
  title: string;
  logo?: {
    alt: string;
    src: string;
  };
}

const ModernNavbar: React.FC<ModernNavbarProps> = ({ items, title, logo }) => {
  const [isMenuOpen, setIsMenuOpen] = useState(false);
  const [activeDropdown, setActiveDropdown] = useState<string | null>(null);
  const location = useLocation();
  const dropdownRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setActiveDropdown(null);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  const toggleMenu = () => {
    setIsMenuOpen(!isMenuOpen);
  };

  const toggleDropdown = (label: string) => {
    setActiveDropdown(activeDropdown === label ? null : label);
  };

  const isActive = (to?: string, href?: string) => {
    if (to && location.pathname.startsWith(to)) {
      return true;
    }
    if (href) {
      return location.pathname.includes(new URL(href, window.location.origin).pathname);
    }
    return false;
  };

  const renderNavItem = (item: NavItem, index: number) => {
    if (item.type === 'dropdown' && item.items) {
      return (
        <div key={index} className={styles.navDropdown} ref={dropdownRef}>
          <button
            className={clsx(styles.navDropdownButton, {
              [styles.navDropdownButtonActive]: activeDropdown === item.label
            })}
            onClick={() => toggleDropdown(item.label)}
            aria-expanded={activeDropdown === item.label}
          >
            {item.label}
            <span className={clsx(styles.navDropdownArrow, {
              [styles.navDropdownArrowActive]: activeDropdown === item.label
            })}>
              ▼
            </span>
          </button>
          <div
            className={clsx(styles.navDropdownMenu, {
              [styles.navDropdownMenuActive]: activeDropdown === item.label
            })}
          >
            {item.items.map((subItem, subIndex) => (
              <Link
                key={subIndex}
                to={subItem.to}
                href={subItem.href}
                className={clsx(styles.navDropdownItem, {
                  [styles.navDropdownItemActive]: isActive(subItem.to, subItem.href)
                })}
                onClick={() => {
                  setActiveDropdown(null);
                  setIsMenuOpen(false);
                }}
              >
                {subItem.label}
              </Link>
            ))}
          </div>
        </div>
      );
    }

    return (
      <Link
        key={index}
        to={item.to}
        href={item.href}
        className={clsx(styles.navLink, {
          [styles.navLinkActive]: isActive(item.to, item.href)
        })}
        onClick={() => setIsMenuOpen(false)}
      >
        {item.label}
      </Link>
    );
  };

  return (
    <nav className={styles.navbar}>
      <div className={styles.navbarContainer}>
        <div className={styles.navbarBrand}>
          {logo && (
            <Link to="/" className={styles.navbarLogo}>
              <img src={logo.src} alt={logo.alt} className={styles.navbarLogoImg} />
            </Link>
          )}
          <Link to="/" className={styles.navbarTitle}>
            {title}
          </Link>
        </div>

        <div className={styles.navbarMenuButton} onClick={toggleMenu}>
          <span className={styles.navbarMenuIcon} />
        </div>

        <div className={clsx(styles.navbarMenu, { [styles.navbarMenuOpen]: isMenuOpen })}>
          <div className={styles.navbarNav}>
            {items.map((item, index) => renderNavItem(item, index))}
          </div>
        </div>
      </div>
    </nav>
  );
};

export default ModernNavbar;