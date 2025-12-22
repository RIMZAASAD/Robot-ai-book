import sys
import os
from pathlib import Path

# Add the backend/src path to Python path so we can import the service
sys.path.insert(0, os.path.join(os.getcwd(), 'src'))

# Get the project root directory (one level up from backend)
project_root = Path(os.getcwd()).parent
textbook_path = project_root / "textbook"

print(f"Project root: {project_root}")
print(f"Textbook path: {textbook_path}")
print(f"Textbook path exists: {textbook_path.exists()}")

if textbook_path.exists():
    # List markdown files in textbook directory
    markdown_files = list(textbook_path.rglob("*.md"))
    print(f"Found {len(markdown_files)} markdown files in textbook")
    for file in markdown_files[:10]:  # Print first 10
        print(f"  - {file}")
else:
    print("Textbook directory does not exist at the expected location")