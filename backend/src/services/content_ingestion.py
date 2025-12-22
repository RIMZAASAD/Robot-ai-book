import os
import re
import logging
from typing import List, Dict, Any
from pathlib import Path
import markdown
from bs4 import BeautifulSoup
import asyncio

logger = logging.getLogger(__name__)

class ContentIngestionService:
    def __init__(self):
        self.supported_extensions = {'.md', '.mdx'}
        self.chunk_size = 1000  # Approximate character count per chunk
        self.overlap_size = 100  # Overlap between chunks to maintain context

    def extract_content_from_markdown(self, file_path: str) -> List[Dict[str, Any]]:
        """
        Extract content from a markdown file, preserving structure
        """
        with open(file_path, 'r', encoding='utf-8') as file:
            content = file.read()

        # Parse the markdown to extract sections
        sections = self._parse_markdown_sections(content, file_path)

        # Chunk the content appropriately
        chunks = self._chunk_content(sections, file_path)

        return chunks

    def _parse_markdown_sections(self, content: str, file_path: str) -> List[Dict[str, Any]]:
        """
        Parse markdown content into sections based on headings
        """
        lines = content.split('\n')
        sections = []
        current_section = {'title': 'Introduction', 'content': '', 'start_line': 0}

        # Extract the filename without extension as a default title
        default_title = Path(file_path).stem.replace('-', ' ').title()

        for i, line in enumerate(lines):
            # Check for markdown headings (h1, h2, h3, etc.)
            heading_match = re.match(r'^(#{1,6})\s+(.+)', line)
            if heading_match:
                # Save the previous section if it has content
                if current_section['content'].strip():
                    sections.append(current_section)

                # Start a new section
                heading_level = len(heading_match.group(1))
                heading_text = heading_match.group(2).strip()

                current_section = {
                    'title': heading_text,
                    'content': f"{heading_text}\n",  # Include heading in content
                    'start_line': i,
                    'heading_level': heading_level
                }
            else:
                current_section['content'] += line + '\n'

        # Add the last section
        if current_section['content'].strip():
            sections.append(current_section)

        # If no headings were found, create one section with the default title
        if not sections and content.strip():
            sections = [{
                'title': default_title,
                'content': content,
                'start_line': 0
            }]

        return sections

    def _chunk_content(self, sections: List[Dict[str, Any]], file_path: str) -> List[Dict[str, Any]]:
        """
        Chunk content into appropriate sizes while preserving semantic boundaries
        """
        chunks = []
        chunk_id = 0

        for section in sections:
            section_content = section['content']
            section_title = section['title']

            # If the section is smaller than chunk size, add as is
            if len(section_content) <= self.chunk_size:
                chunk = {
                    'id': f"{Path(file_path).stem}_chunk_{chunk_id}",
                    'content': section_content.strip(),
                    'title': section_title,
                    'source_file': file_path,
                    'section': section_title
                }
                chunks.append(chunk)
                chunk_id += 1
            else:
                # Split the section into smaller chunks
                section_chunks = self._split_large_section(section_content, section_title, file_path)
                for chunk_content in section_chunks:
                    chunk = {
                        'id': f"{Path(file_path).stem}_chunk_{chunk_id}",
                        'content': chunk_content.strip(),
                        'title': section_title,
                        'source_file': file_path,
                        'section': section_title
                    }
                    chunks.append(chunk)
                    chunk_id += 1

        return chunks

    def _split_large_section(self, content: str, title: str, file_path: str) -> List[str]:
        """
        Split a large section into smaller chunks while maintaining context
        """
        chunks = []

        # Split by paragraphs first
        paragraphs = content.split('\n\n')

        current_chunk = ""
        for paragraph in paragraphs:
            # If adding this paragraph would exceed the chunk size
            if len(current_chunk) + len(paragraph) > self.chunk_size and current_chunk:
                # Add the current chunk to the list
                chunks.append(current_chunk.strip())

                # Start a new chunk, potentially with overlap
                if self.overlap_size > 0 and len(paragraph) > self.overlap_size:
                    # Add overlap from the end of the previous chunk
                    overlap = current_chunk[-self.overlap_size:]
                    current_chunk = overlap + "\n\n" + paragraph
                else:
                    current_chunk = paragraph
            else:
                # Add paragraph to current chunk
                if current_chunk:
                    current_chunk += "\n\n" + paragraph
                else:
                    current_chunk = paragraph

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    def get_all_markdown_files(self, textbook_path: str = None) -> List[str]:
        """
        Recursively find all markdown files in the textbook directory
        """
        # Default to relative path from the project structure
        if textbook_path is None:
            # Try multiple possible paths based on where the service might be running from
            possible_paths = [
                "../../../textbook",  # Path when running from backend/src/services/
                "../textbook",        # Path when running from backend/src/
                "../../textbook",     # Path when running from backend/
                "textbook",           # Path when running from project root
            ]

            # Add absolute path based on the service file location
            service_file_dir = Path(__file__).parent.parent.parent.parent  # Go up 4 levels to project root
            absolute_path = service_file_dir / "textbook"
            possible_paths.append(str(absolute_path))

            textbook_dir = None
            for path in possible_paths:
                textbook_dir = Path(path)
                if textbook_dir.exists():
                    logger.info(f"Found textbook directory at: {textbook_dir.absolute()}")
                    break
        else:
            textbook_dir = Path(textbook_path)

        if textbook_dir is None or not textbook_dir.exists():
            logger.error(f"Textbook directory not found. Tried paths: {possible_paths}")
            return []

        markdown_files = []
        for ext in self.supported_extensions:
            markdown_files.extend(textbook_dir.rglob(f"*{ext}"))

        return [str(f) for f in markdown_files]

    async def ingest_textbook_content(self) -> List[Dict[str, Any]]:
        """
        Ingest all textbook content and return chunks ready for embedding
        """
        logger.info("Starting textbook content ingestion...")

        # Get all markdown files
        markdown_files = self.get_all_markdown_files()
        logger.info(f"Found {len(markdown_files)} markdown files to process")

        all_chunks = []
        for file_path in markdown_files:
            try:
                logger.info(f"Processing file: {file_path}")
                chunks = self.extract_content_from_markdown(file_path)

                # Add additional metadata to each chunk
                for chunk in chunks:
                    chunk['chapter'] = Path(file_path).parent.name
                    chunk['created_at'] = str(chunks[0]['start_line'] if 'start_line' in chunks[0] else 0)  # Placeholder timestamp

                all_chunks.extend(chunks)
                logger.info(f"Extracted {len(chunks)} chunks from {file_path}")

            except Exception as e:
                logger.error(f"Error processing file {file_path}: {str(e)}")
                continue

        logger.info(f"Completed ingestion. Total chunks: {len(all_chunks)}")
        return all_chunks

# Global instance
content_ingestion_service = ContentIngestionService()