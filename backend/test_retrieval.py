"""Test retrieval service directly"""
import asyncio
from src.services.retrieval_service import RetrievalService

async def test_retrieval():
    print("🔍 Testing retrieval service...\n")
    
    retrieval_service = RetrievalService()
    
    query = "What is Physical AI?"
    print(f"Query: {query}\n")
    
    try:
        results = await retrieval_service.retrieve_chunks(
            query_text=query,
            top_k=5,
            similarity_threshold=0.5
        )
        
        print(f"✅ Retrieved {len(results)} chunks\n")
        
        if results:
            for i, chunk in enumerate(results, 1):
                print(f"--- Chunk {i} ---")
                print(f"Score: {chunk.similarity_score}")
                print(f"Content: {chunk.content[:100]}...")
                print(f"Source: {chunk.source_url}")
                print()
        else:
            print("⚠️ No chunks retrieved!")
            print("\nTrying with lower threshold...")
            
            results = await retrieval_service.retrieve_chunks(
                query_text=query,
                top_k=5,
                similarity_threshold=0.0
            )
            
            print(f"Retrieved {len(results)} chunks with threshold=0.0")
            if results:
                print(f"Top score: {results[0].similarity_score}")
                
    except Exception as e:
        print(f"❌ Error: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test_retrieval())
