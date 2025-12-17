import sys
import os
from pathlib import Path

# Add the backend/src path to Python path so we can import the service
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

from services.content_ingestion import content_ingestion_service

print("Current working directory:", os.getcwd())
print("Looking for textbook directory...")

# Test the path resolution
textbook_path = "../../../textbook"
textbook_dir = Path(textbook_path)

print(f"Textbook path: {textbook_path}")
print(f"Textbook dir exists: {textbook_dir.exists()}")
print(f"Absolute path: {textbook_dir.absolute()}")

# Try the alternative path used in the service
alt_path = "../../../textbook"
alt_dir = Path(alt_path)
print(f"Alternative path: {alt_path}")
print(f"Alternative dir exists: {alt_dir.exists()}")

# Get all markdown files like the service does
markdown_files = content_ingestion_service.get_all_markdown_files()
print(f"Found {len(markdown_files)} markdown files")
for file in markdown_files[:5]:  # Print first 5
    print(f"  - {file}")