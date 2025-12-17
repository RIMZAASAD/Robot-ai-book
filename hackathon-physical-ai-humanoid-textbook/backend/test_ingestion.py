import sys
import os
import asyncio
from pathlib import Path

# Add the backend/src path to Python path so we can import the service
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

from services.content_ingestion import content_ingestion_service

async def test_ingestion():
    print("Testing content ingestion...")

    # Test getting markdown files
    markdown_files = content_ingestion_service.get_all_markdown_files()
    print(f"Found {len(markdown_files)} markdown files using service method")

    for file in markdown_files[:5]:  # Print first 5
        print(f"  - {file}")

    # Test the full ingestion process
    try:
        chunks = await content_ingestion_service.ingest_textbook_content()
        print(f"Successfully extracted {len(chunks)} content chunks")

        # Print info about first few chunks
        for i, chunk in enumerate(chunks[:3]):
            print(f"Chunk {i+1}: {chunk.get('title', 'No title')} from {chunk.get('source_file', 'Unknown file')}")

    except Exception as e:
        print(f"Error during ingestion: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test_ingestion())