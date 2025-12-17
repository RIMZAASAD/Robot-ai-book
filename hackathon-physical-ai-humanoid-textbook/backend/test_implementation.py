"""
Simple test script to verify the RAG Chatbot implementation
"""
import asyncio
import os
from src.services.embedding_service import embedding_service
from src.services.qdrant_service import qdrant_service
from src.services.openrouter_service import openrouter_service

async def test_implementation():
    print("Testing RAG Chatbot Implementation...")

    # Test 1: Embedding service
    print("\n1. Testing Embedding Service...")
    try:
        test_text = "This is a test sentence for embedding."
        embedding = await embedding_service.generate_embedding(test_text)
        print(f"✓ Generated embedding of length: {len(embedding)}")
        print(f"First 5 values: {embedding[:5]}")
    except Exception as e:
        print(f"✗ Embedding service test failed: {e}")
        return

    # Test 2: Qdrant service (basic connectivity)
    print("\n2. Testing Qdrant Service...")
    try:
        # This will test if we can access the collection
        collections = qdrant_service.client.get_collections()
        print(f"✓ Connected to Qdrant, found {len(collections.collections)} collections")
    except Exception as e:
        print(f"✗ Qdrant service test failed: {e}")
        return

    # Test 3: OpenRouter service (basic connectivity)
    print("\n3. Testing OpenRouter Service...")
    try:
        messages = [
            {"role": "user", "content": "Hello, this is a test. Just respond with 'Test successful'."}
        ]
        response = await openrouter_service.generate_response(messages, max_tokens=20)
        print(f"✓ OpenRouter service test successful: '{response[:50]}...'")
    except Exception as e:
        print(f"✗ OpenRouter service test failed: {e}")
        return

    # Test 4: Simple end-to-end test (without actual storage)
    print("\n4. Testing end-to-end flow...")
    try:
        # Generate embedding for a test question
        question = "What is ROS?"
        question_embedding = await embedding_service.generate_embedding(question)
        print(f"✓ Generated embedding for question: '{question}'")

        # Test search (this will return empty if no data is stored yet)
        results = await qdrant_service.search_similar(question_embedding, limit=2)
        print(f"✓ Search completed, found {len(results)} results")

        # Test LLM response generation
        messages = [
            {"role": "user", "content": "What is 2+2? Respond with just the number."}
        ]
        response = await openrouter_service.generate_response(messages, max_tokens=10)
        print(f"✓ LLM response test: '{response}'")

    except Exception as e:
        print(f"✗ End-to-end test failed: {e}")
        return

    print("\n✓ All tests passed! Implementation is working correctly.")

if __name__ == "__main__":
    # Check if required environment variables are set
    required_vars = ["OPENROUTER_API_KEY", "QDRANT_URL"]
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        print(f"Warning: Missing environment variables: {missing_vars}")
        print("Please set these variables before running the full application.")
    else:
        asyncio.run(test_implementation())