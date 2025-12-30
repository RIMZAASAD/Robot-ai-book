"""Check Qdrant collection status"""
import asyncio
from src.config.qdrant_config import QdrantConfig

async def check_collection():
    print("🔍 Checking Qdrant collection...")
    
    qdrant_config = QdrantConfig()
    client = qdrant_config.get_client()
    
    # Get collection info
    collection_name = "textbook_content"
    
    try:
        collection_info = client.get_collection(collection_name)
        
        print(f"\n✅ Collection: {collection_name}")
        print(f"   Points count: {collection_info.points_count}")
        print(f"   Vector size: {collection_info.config.params.vectors.size}")
        
        if collection_info.points_count == 0:
            print("\n⚠️ WARNING: Collection is EMPTY!")
            print("   Textbook content has not been ingested yet.")
            print("\n💡 Solution: Run the ingestion script to populate the database:")
            print("   python -m src.scripts.ingest_textbook")
        else:
            print(f"\n✅ Collection has {collection_info.points_count} chunks stored")
            
            # Try a sample search
            print("\n🔍 Testing search with sample query...")
            from src.services.embedding_service import EmbeddingService
            
            embedding_service = EmbeddingService()
            query_embedding = await embedding_service.execute(["What is Physical AI?"], input_type="search_query")
            
            results = client.search(
                collection_name=collection_name,
                query_vector=query_embedding[0],
                limit=3
            )
            
            if results:
                print(f"✅ Search working! Found {len(results)} results")
                print(f"   Top result score: {results[0].score}")
                print(f"   Content preview: {results[0].payload.get('content', '')[:100]}...")
            else:
                print("⚠️ Search returned no results - might be threshold issue")
                
    except Exception as e:
        print(f"\n❌ Error: {str(e)}")
        if "not found" in str(e).lower():
            print("\n💡 Collection doesn't exist. Need to create it first.")

if __name__ == "__main__":
    asyncio.run(check_collection())
