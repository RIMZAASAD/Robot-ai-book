import asyncio
import sys
import os
from dotenv import load_dotenv

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__)))

from src.services.vector_search import VectorSearchService
from src.services.embedding_service import EmbeddingService
from src.services.llm_service import LLMService

async def ask_question(query: str):
    print(f"\nSearching for: '{query}'...")
    
    # Initialize services
    embedding_service = EmbeddingService()
    search_service = VectorSearchService()
    llm_service = LLMService()
    
    # Step 1: Embed query
    embeddings = await embedding_service.execute([query])
    query_vector = embeddings[0]
    
    # Step 2: Search Qdrant
    # Using a low threshold for better retrieval of natural language queries
    chunks = search_service.search_similar_chunks(query_vector, top_k=3, threshold=0.1)
    
    if not chunks:
        print("No relevant context found in Qdrant.")
        return
        
    print(f"Found {len(chunks)} relevant chunks. Generating answer...\n")
    
    # Step 3: Generate Answer
    answer = await llm_service.generate_answer(query, chunks)
    
    print("-" * 50)
    print("AI ANSWER:")
    print("-" * 50)
    print(answer)
    print("-" * 50)
    print("\nSOURCES:")
    for i, c in enumerate(chunks):
        url = c.get('source_url', 'N/A')
        score = c.get('score', 0)
        print(f"[{i+1}] {url} (Similarity: {score:.4f})")

if __name__ == "__main__":
    load_dotenv()
    if len(sys.argv) > 1:
        query = " ".join(sys.argv[1:])
    else:
        query = input("Ask a question about Physical AI / Robotics: ")
    
    asyncio.run(ask_question(query))
