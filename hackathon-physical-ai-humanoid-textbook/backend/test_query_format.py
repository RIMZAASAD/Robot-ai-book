import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
import asyncio

# Load environment variables
load_dotenv()

# Get Qdrant configuration from environment
url = os.getenv("QDRANT_URL")
api_key = os.getenv("QDRANT_API_KEY")

# Initialize Qdrant client
client = QdrantClient(
    url=url,
    api_key=api_key,
    timeout=10.0
)

collection_name = "textbook_content"

# Test with a simple embedding (zeros for testing)
test_embedding = [0.0] * 384

try:
    # Test query_points to see the response format
    print("Testing query_points response format...")
    result = client.query_points(
        collection_name=collection_name,
        query=test_embedding,
        limit=1,
        with_payload=True,
        with_vectors=False
    )

    print(f"Result type: {type(result)}")
    print(f"Result content: {result}")

    if hasattr(result, '__iter__') and not isinstance(result, str):
        print(f"Result is iterable with {len(result) if hasattr(result, '__len__') else 'unknown length'} items")
        if len(result) > 0:
            first_item = result[0] if isinstance(result, list) else list(result)[0] if hasattr(result, '__iter__') else result
            print(f"First item type: {type(first_item)}")
            print(f"First item: {first_item}")
    else:
        print("Result is not a list-like object")

    # Check if it has attributes like a ScoredPoint
    if hasattr(result, 'points'):
        print(f"Result has 'points' attribute with {len(result.points)} items")
        if result.points:
            print(f"First point: {result.points[0]}")
            print(f"First point type: {type(result.points[0])}")
            print(f"First point attributes: {dir(result.points[0])}")

except Exception as e:
    print(f"Error testing query_points: {e}")
    import traceback
    traceback.print_exc()