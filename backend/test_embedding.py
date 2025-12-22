import asyncio
import sys
import os

# Add the src directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from services.embedding_service import embedding_service

async def test_embedding():
    print("Testing embedding service...")

    try:
        # Test single embedding
        text = "This is a test sentence for embedding."
        embedding = await embedding_service.generate_embedding(text)
        print(f"Generated embedding of length: {len(embedding)}")
        print(f"First 5 values: {embedding[:5]}")

        # Test multiple embeddings
        texts = ["First sentence", "Second sentence", "Third sentence"]
        embeddings = await embedding_service.generate_embeddings(texts)
        print(f"Generated {len(embeddings)} embeddings")
        print(f"Each embedding has length: {len(embeddings[0]) if embeddings else 0}")

        print("Embedding service test completed successfully!")

    except Exception as e:
        print(f"Embedding service test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test_embedding())