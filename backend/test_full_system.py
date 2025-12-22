import asyncio
import os
import sys
from dotenv import load_dotenv

# Add the src directory to the path so we can import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

# Load environment variables
load_dotenv()

from services.qdrant_service import qdrant_service
from services.postgres_service import postgres_service
from services.embedding_service import embedding_service
from services.openrouter_service import openrouter_service

async def test_full_system():
    print("Testing full system integration...")

    # Test 1: Qdrant Service
    print("\n1. Testing Qdrant Service...")
    try:
        collections = qdrant_service.client.get_collections()
        print("+ Connected to Qdrant cloud instance")
        print(f"+ Found {len(collections.collections)} collections")
    except Exception as e:
        print(f"- Qdrant service test failed: {e}")
        return

    # Test 2: PostgreSQL Service (in-memory)
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

    # Test 3: Embedding Service
    print("\n3. Testing Embedding Service...")
    try:
        test_text = "This is a test for embedding functionality."
        embedding = await embedding_service.generate_embedding(test_text)
        print(f"+ Generated embedding of length: {len(embedding)}")
        print(f"+ First 5 values: {embedding[:5]}")

        # Test multiple embeddings
        texts = ["First sentence", "Second sentence"]
        embeddings = await embedding_service.generate_embeddings(texts)
        print(f"+ Generated {len(embeddings)} embeddings successfully")
    except Exception as e:
        print(f"- Embedding service test failed: {e}")
        return

    # Test 4: OpenRouter Service
    print("\n4. Testing OpenRouter Service...")
    try:
        messages = [
            {"role": "user", "content": "Hello, this is a test. Just respond with 'Test successful'."}
        ]
        response = await openrouter_service.generate_response(messages, max_tokens=20)
        print(f"+ OpenRouter service test successful: '{response[:50]}...'")
    except Exception as e:
        print(f"- OpenRouter service test failed: {e}")
        return

    # Test 5: End-to-end RAG simulation
    print("\n5. Testing end-to-end RAG simulation...")
    try:
        # Generate embedding for a question
        question = "What is ROS in robotics?"
        question_embedding = await embedding_service.generate_embedding(question)
        print(f"+ Generated embedding for question: '{question}'")

        # Try to search in Qdrant (will return empty if no data is stored yet)
        try:
            results = await qdrant_service.search_similar(question_embedding, limit=2)
            print(f"+ Search completed, found {len(results)} results")
        except Exception as e:
            print(f"~ Search test had issue (expected if no data ingested yet): {e}")

        # Generate response using context (simulated)
        context = "Robot Operating System (ROS) is a flexible framework for writing robot software."
        messages = [
            {
                "role": "system",
                "content": "You are an assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based on the provided context."
            },
            {
                "role": "user",
                "content": f"Context: {context}\n\nQuestion: {question}\n\nPlease provide an answer based on the context provided."
            }
        ]
        response = await openrouter_service.generate_response(messages)
        print(f"+ RAG response generated: '{response[:50]}...'")

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
    asyncio.run(test_full_system())