import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

interface SearchSuggestion {
  title: string;
  description: string;
  url: string;
}

interface ModernSearchProps {
  className?: string;
  onSearch?: (query: string) => void;
  suggestions?: SearchSuggestion[];
  placeholder?: string;
}

const ModernSearch: React.FC<ModernSearchProps> = ({
  className = '',
  onSearch,
  suggestions = [],
  placeholder = 'Search documentation...'
}) => {
  const [query, setQuery] = useState('');
  const [isOpen, setIsOpen] = useState(false);
  const [selectedIndex, setSelectedIndex] = useState(-1);
  const searchRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (searchRef.current && !searchRef.current.contains(event.target as Node)) {
        setIsOpen(false);
        setSelectedIndex(-1);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const value = e.target.value;
    setQuery(value);
    setIsOpen(value.length > 0);
    setSelectedIndex(-1);
    onSearch?.(value);
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'ArrowDown') {
      e.preventDefault();
      setSelectedIndex(prev => Math.min(prev + 1, suggestions.length - 1));
    } else if (e.key === 'ArrowUp') {
      e.preventDefault();
      setSelectedIndex(prev => Math.max(prev - 1, -1));
    } else if (e.key === 'Enter' && selectedIndex >= 0) {
      e.preventDefault();
      window.location.href = suggestions[selectedIndex].url;
    }
  };

  const handleSuggestionClick = (url: string) => {
    window.location.href = url;
    setIsOpen(false);
    setQuery('');
  };

  return (
    <div className={clsx(styles.searchContainer, className)} ref={searchRef}>
      <div className={styles.searchWrapper}>
        <input
          type="text"
          value={query}
          onChange={handleInputChange}
          onKeyDown={handleKeyDown}
          onFocus={() => query && setIsOpen(true)}
          placeholder={placeholder}
          className={styles.searchInput}
          aria-label="Search documentation"
        />
        <div className={styles.searchIcon}>🔍</div>
      </div>

      {isOpen && suggestions.length > 0 && (
        <div className={styles.searchDropdown}>
          {suggestions.map((suggestion, index) => (
            <div
              key={index}
              className={clsx(styles.searchSuggestion, {
                [styles.searchSuggestionActive]: index === selectedIndex
              })}
              onClick={() => handleSuggestionClick(suggestion.url)}
              onMouseEnter={() => setSelectedIndex(index)}
            >
              <div className={styles.suggestionTitle}>{suggestion.title}</div>
              <div className={styles.suggestionDescription}>{suggestion.description}</div>
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

export default ModernSearch;