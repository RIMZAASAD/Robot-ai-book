import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams

# Load environment variables
load_dotenv()

# Get Qdrant configuration from environment
url = os.getenv("QDRANT_URL")
api_key = os.getenv("QDRANT_API_KEY")

if not url or not api_key:
    print("Please set QDRANT_URL and QDRANT_API_KEY environment variables")
    exit(1)

# Initialize Qdrant client
client = QdrantClient(
    url=url,
    api_key=api_key,
    timeout=10.0
)

collection_name = "textbook_content"

try:
    # Delete existing collection
    print(f"Deleting existing collection: {collection_name}")
    client.delete_collection(collection_name)
    print(f"Collection {collection_name} deleted successfully")

    # Create new collection with correct dimensions
    print(f"Creating new collection: {collection_name} with 384 dimensions")
    client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(size=384, distance=Distance.COSINE)
    )
    print(f"Collection {collection_name} created successfully with 384 dimensions")

except Exception as e:
    print(f"Error managing Qdrant collection: {e}")