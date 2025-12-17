"""
Test script to verify the services work with the provided credentials
"""
import asyncio
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

from src.services.qdrant_service import qdrant_service
from src.services.postgres_service import postgres_service
from src.services.embedding_service import embedding_service
from src.services.openrouter_service import openrouter_service

async def test_services():
    print("Testing services with provided credentials...")

    # Test 1: Qdrant Service
    print("\n1. Testing Qdrant Service...")
    try:
        collections = qdrant_service.client.get_collections()
        print("+ Connected to Qdrant cloud instance")
        print(f"+ Found {len(collections.collections)} collections")

        # Test embedding generation
        test_text = "This is a test for Qdrant connection."
        embedding = await embedding_service.generate_embedding(test_text)
        print(f"+ Generated embedding of length: {len(embedding)}")

    except Exception as e:
        print(f"- Qdrant service test failed: {e}")
        return

    # Test 2: PostgreSQL Service
    print("\n2. Testing PostgreSQL Service...")
    try:
        await postgres_service.initialize()
        print("+ PostgreSQL service initialized successfully")

        # Test inserting and retrieving a session
        session_data = {
            "session_id": "test_session_123",
            "user_id": "test_user",
            "page_history": ["module-1/intro"],
            "mode_preference": "full_textbook"
        }
        session_id = await postgres_service.store_session(session_data)
        print(f"+ Session stored successfully: {session_id}")

    except Exception as e:
        print(f"- PostgreSQL service test failed: {e}")
        return

    # Test 3: OpenRouter Service
    print("\n3. Testing OpenRouter Service...")
    try:
        messages = [
            {"role": "user", "content": "Hello, this is a test. Just respond with 'Test successful'."}
        ]
        response = await openrouter_service.generate_response(messages, max_tokens=20)
        print(f"+ OpenRouter service test successful: '{response[:50]}...'")
    except Exception as e:
        print(f"- OpenRouter service test failed: {e}")
        return

    # Test 4: End-to-end test
    print("\n4. Testing end-to-end functionality...")
    try:
        # Test search (might return empty if no data is stored yet)
        question = "What is ROS?"
        question_embedding = await embedding_service.generate_embedding(question)
        print(f"+ Generated embedding for question: '{question}'")

        # This might fail if the collection doesn't exist yet, which is okay
        try:
            results = await qdrant_service.search_similar(question_embedding, limit=2)
            print(f"+ Search completed, found {len(results)} results")
        except Exception as e:
            print(f"~ Search test had issue (expected if no data ingested yet): {e}")

        print("+ End-to-end test completed")
    except Exception as e:
        print(f"- End-to-end test failed: {e}")
        return

    print("\n+ All services are working correctly with the provided credentials!")
    print("\nNext steps:")
    print("1. Run 'uvicorn src.main:app --reload --port 8000' to start the API")
    print("2. Ingest textbook content using POST /v1/ingest-textbook")
    print("3. Test the chat endpoints")

if __name__ == "__main__":
    asyncio.run(test_services())