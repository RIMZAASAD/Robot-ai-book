import argparse
import asyncio
import sys
import os
from typing import List, Dict, Any
import logging

# Add the backend directory to Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from dotenv import load_dotenv
from ..services.url_fetching import URLFetchingService
from ..services.html_extraction import HTMLExtractionService
from ..services.text_cleaning import TextCleaningService
from ..services.text_chunking import TextChunkingService
from ..services.embedding_service import EmbeddingService
from ..services.vector_storage import VectorStorageService
from ..config.qdrant_config import QdrantConfig
from ..config.settings import settings


class IngestionCLI:
    """
    Command-line interface for the RAG ingestion pipeline
    """

    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        load_dotenv()  # Load environment variables

        # Initialize services
        self.url_fetching_service = URLFetchingService()
        self.html_extraction_service = HTMLExtractionService()
        self.text_cleaning_service = TextCleaningService()
        self.text_chunking_service = TextChunkingService()
        self.embedding_service = EmbeddingService()
        self.vector_storage_service = VectorStorageService()
        self.qdrant_config = QdrantConfig()

    async def process_urls(self, urls: List[str], job_name: str = "cli_ingestion",
                          chunk_size: int = 512, chunk_overlap: int = 102) -> Dict[str, Any]:
        """
        Process a list of URLs through the entire ingestion pipeline
        """
        self.logger.info(f"Starting CLI ingestion job: {job_name} with {len(urls)} URLs")

        # Initialize Qdrant
        self.qdrant_config.initialize_client()
        self.qdrant_config.initialize_collection()

        total_documents = 0
        total_chunks = 0
        total_vectors = 0

        all_embeddings = []
        all_texts = []
        all_metadata = []

        for url in urls:
            self.logger.info(f"Processing URL: {url}")

            try:
                # Step 1: Fetch content from URL
                fetched_data = await self.url_fetching_service.fetch_content(url)
                self.logger.info(f"Fetched content from {url}, length: {len(fetched_data['content'])}")

                # Step 2: Extract clean text from HTML
                extracted_data = await self.html_extraction_service.execute(fetched_data['content'], url)
                if not extracted_data.get("success", True):
                    self.logger.error(f"Failed to extract content from {url}: {extracted_data.get('error')}")
                    continue

                # Step 3: Clean the text
                cleaned_data = await self.text_cleaning_service.execute(extracted_data["content"])
                if not cleaned_data.get("success", True):
                    self.logger.error(f"Failed to clean text from {url}: {cleaned_data.get('error')}")
                    continue

                # Step 4: Chunk the text
                chunks = await self.text_chunking_service.execute(
                    cleaned_data["cleaned_text"],
                    chunk_size=chunk_size,
                    overlap=chunk_overlap
                )
                self.logger.info(f"Created {len(chunks)} chunks from {url}")

                # Prepare chunks for embedding
                for i, chunk in enumerate(chunks):
                    metadata = {
                        "title": extracted_data["title"],
                        "url": url,
                        "chunk_index": i,
                        "document_id": f"doc_{total_documents}",
                        "created_at": "",
                        "source_type": "web_page"
                    }
                    all_metadata.append(metadata)
                    all_texts.append(chunk["content"])

                total_documents += 1
                total_chunks += len(chunks)

            except Exception as e:
                self.logger.error(f"Error processing {url}: {str(e)}")
                continue

        if all_texts:
            # Step 5: Generate embeddings
            self.logger.info(f"Generating embeddings for {len(all_texts)} text chunks")
            embeddings = await self.embedding_service.execute(all_texts)
            all_embeddings.extend(embeddings)

            # Step 6: Store embeddings in Qdrant
            self.logger.info(f"Storing {len(embeddings)} embeddings in Qdrant")
            stored_ids = await self.vector_storage_service.execute(embeddings, all_texts, all_metadata)
            total_vectors = len(stored_ids)

        result = {
            "job_name": job_name,
            "documents_processed": total_documents,
            "chunks_created": total_chunks,
            "vectors_stored": total_vectors,
            "status": "completed" if total_documents > 0 else "failed"
        }

        self.logger.info(f"CLI ingestion completed: {result}")
        return result

    def run(self):
        """
        Run the CLI with command-line arguments
        """
        parser = argparse.ArgumentParser(description="RAG Data Ingestion Pipeline CLI")
        parser.add_argument("--urls", nargs="+", required=True, help="URLs to process")
        parser.add_argument("--job-name", default="cli_ingestion", help="Name for the ingestion job")
        parser.add_argument("--chunk-size", type=int, default=512, help="Size of text chunks")
        parser.add_argument("--chunk-overlap", type=int, default=102, help="Overlap between chunks")

        args = parser.parse_args()

        # Set up logging
        logging.basicConfig(level=logging.INFO)

        # Run the ingestion pipeline
        result = asyncio.run(self.process_urls(
            urls=args.urls,
            job_name=args.job_name,
            chunk_size=args.chunk_size,
            chunk_overlap=args.chunk_overlap
        ))

        # Print results
        print(f"\nIngestion completed successfully!")
        print(f"Job: {result['job_name']}")
        print(f"Documents processed: {result['documents_processed']}")
        print(f"Chunks created: {result['chunks_created']}")
        print(f"Vectors stored: {result['vectors_stored']}")
        print(f"Status: {result['status']}")


def main():
    """
    Main function to run the CLI
    """
    cli = IngestionCLI()
    cli.run()


if __name__ == "__main__":
    main()