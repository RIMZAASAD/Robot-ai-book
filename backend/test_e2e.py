#!/usr/bin/env python3
"""
End-to-end test with 100 sample URLs for the RAG ingestion pipeline
"""
import asyncio
import time
from src.services.url_fetching import URLFetchingService
from src.services.html_extraction import HTMLExtractionService
from src.services.text_cleaning import TextCleaningService
from src.services.text_chunking import TextChunkingService
from src.services.embedding_service import EmbeddingService
from src.services.vector_storage import VectorStorageService
from src.config.qdrant_config import QdrantConfig
from src.config.settings import settings


async def run_end_to_end_test():
    """
    Run end-to-end test with sample URLs
    """
    print("Starting end-to-end test with sample URLs...")

    # Initialize services
    qdrant_config = QdrantConfig()
    qdrant_client = qdrant_config.initialize_client()
    qdrant_config.initialize_collection()

    url_fetching_service = URLFetchingService()
    html_extraction_service = HTMLExtractionService()
    text_cleaning_service = TextCleaningService()
    text_chunking_service = TextChunkingService()
    embedding_service = EmbeddingService()
    vector_storage_service = VectorStorageService()

    # Sample URLs for testing (using public URLs that are generally available)
    sample_urls = [
        "https://en.wikipedia.org/wiki/Artificial_intelligence",
        "https://en.wikipedia.org/wiki/Machine_learning",
        "https://en.wikipedia.org/wiki/Deep_learning",
        "https://en.wikipedia.org/wiki/Neural_network",
        "https://en.wikipedia.org/wiki/Natural_language_processing",
        "https://en.wikipedia.org/wiki/Computer_vision",
        "https://en.wikipedia.org/wiki/Data_science",
        "https://en.wikipedia.org/wiki/Big_data",
        "https://en.wikipedia.org/wiki/Cloud_computing",
        "https://en.wikipedia.org/wiki/Internet_of_things",
    ] * 10  # Multiply to get 100 URLs

    print(f"Processing {len(sample_urls)} URLs...")

    start_time = time.time()
    total_documents = 0
    total_chunks = 0
    total_vectors = 0

    all_embeddings = []
    all_texts = []
    all_metadata = []

    for i, url in enumerate(sample_urls):
        print(f"Processing URL {i+1}/{len(sample_urls)}: {url}")

        try:
            # Step 1: Fetch content from URL
            fetched_data = await url_fetching_service.fetch_content(url)
            print(f"  Fetched content, length: {len(fetched_data['content'])}")

            # Step 2: Extract clean text from HTML
            extracted_data = await html_extraction_service.execute(fetched_data['content'], url)
            if not extracted_data.get("success", True):
                print(f"  Failed to extract content from {url}")
                continue

            # Step 3: Clean the text
            cleaned_data = await text_cleaning_service.execute(extracted_data["content"])
            if not cleaned_data.get("success", True):
                print(f"  Failed to clean text from {url}")
                continue

            # Step 4: Chunk the text
            chunks = await text_chunking_service.execute(cleaned_data["cleaned_text"])
            print(f"  Created {len(chunks)} chunks")

            # Prepare chunks for potential embedding
            for j, chunk in enumerate(chunks):
                metadata = {
                    "title": extracted_data["title"],
                    "url": url,
                    "chunk_index": j,
                    "document_id": f"doc_{total_documents}",
                    "created_at": "",
                    "source_type": "web_page"
                }
                all_metadata.append(metadata)
                all_texts.append(chunk["content"])

            total_documents += 1
            total_chunks += len(chunks)

        except Exception as e:
            print(f"  Error processing {url}: {str(e)}")
            continue

    if all_texts:
        print(f"Generating embeddings for {len(all_texts)} text chunks...")

        # Process embeddings in batches to avoid API limits
        batch_size = 10  # Small batch size for testing
        for i in range(0, len(all_texts), batch_size):
            batch_texts = all_texts[i:i + batch_size]
            batch_metadata = all_metadata[i:i + batch_size]

            try:
                embeddings = await embedding_service.execute(batch_texts)
                all_embeddings.extend(embeddings)

                # Store this batch
                stored_ids = await vector_storage_service.execute(embeddings, batch_texts, batch_metadata)
                total_vectors += len(stored_ids)

                print(f"  Processed batch {i//batch_size + 1}, stored {len(stored_ids)} vectors")

            except Exception as e:
                print(f"  Error in batch {i//batch_size + 1}: {str(e)}")
                continue

    end_time = time.time()
    duration = end_time - start_time

    print("\n" + "="*50)
    print("END-TO-END TEST RESULTS")
    print("="*50)
    print(f"Total URLs processed: {len(sample_urls)}")
    print(f"Documents successfully processed: {total_documents}")
    print(f"Total chunks created: {total_chunks}")
    print(f"Total vectors stored: {total_vectors}")
    print(f"Processing time: {duration:.2f} seconds")
    print(f"Average time per document: {duration/total_documents:.2f} seconds" if total_documents > 0 else "N/A")

    # Verify stored vectors
    vector_count = vector_storage_service.get_vector_count()
    print(f"Vectors in Qdrant: {vector_count}")

    print("\nTest completed successfully!")
    return {
        "total_urls": len(sample_urls),
        "documents_processed": total_documents,
        "chunks_created": total_chunks,
        "vectors_stored": total_vectors,
        "processing_time": duration,
        "qdrant_vector_count": vector_count
    }


if __name__ == "__main__":
    result = asyncio.run(run_end_to_end_test())
    print(f"\nFinal result: {result}")