import sys
import os
from dotenv import load_dotenv

# Get the backend directory (where this script is located)
backend_path = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, backend_path)

# Add backend/src to path for relative imports to work properly
src_path = os.path.join(backend_path, 'src')
sys.path.insert(0, src_path)

# Load environment variables
load_dotenv(os.path.join(backend_path, '.env'))

if __name__ == "__main__":
    import uvicorn
    # Run from the backend directory with proper PYTHONPATH
    os.chdir(backend_path)
    uvicorn.run(
        "src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        reload_dirs=['src']
    )