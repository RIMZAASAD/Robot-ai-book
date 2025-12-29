"""
Comprehensive test for the RAG Agent Construction feature.

This module contains tests for the agent functionality including
end-to-end testing with 50 sample queries.
"""

import asyncio
import time
import logging
from typing import List, Dict, Any
from src.models.agent_models import UserQuery
from src.agents.rag_agent import RAGAgent
from src.api.v1.agent import query_agent
from src.api.v1.models.agent_models import AgentQueryRequest


# Sample queries for testing the agent
SAMPLE_QUERIES = [
    "What is artificial intelligence?",
    "Explain machine learning concepts",
    "How does natural language processing work?",
    "What are neural networks?",
    "Describe computer vision applications",
    "What is deep learning?",
    "Explain the difference between AI and machine learning",
    "What are the main types of machine learning?",
    "How does supervised learning work?",
    "What is unsupervised learning?",
    "Explain reinforcement learning",
    "What are the applications of AI in healthcare?",
    "How is AI used in autonomous vehicles?",
    "What are the ethical considerations in AI?",
    "Explain the concept of overfitting in machine learning",
    "What is the bias-variance tradeoff?",
    "How do decision trees work?",
    "What is a random forest?",
    "Explain support vector machines",
    "What are k-means clustering algorithms?",
    "How does the k-nearest neighbors algorithm work?",
    "What is the Naive Bayes classifier?",
    "Explain logistic regression",
    "What are the types of neural network architectures?",
    "How does backpropagation work?",
    "What is gradient descent?",
    "Explain the concept of activation functions",
    "What are convolutional neural networks?",
    "How do recurrent neural networks work?",
    "What is the transformer architecture?",
    "Explain attention mechanisms in neural networks",
    "What are generative adversarial networks?",
    "How does natural language understanding work?",
    "What is the difference between NLP and NLU?",
    "Explain word embeddings",
    "What are the applications of computer vision?",
    "How does object detection work?",
    "What is image segmentation?",
    "Explain the concept of transfer learning",
    "What is few-shot learning?",
    "How does ensemble learning work?",
    "What are the challenges in AI development?",
    "Explain the concept of explainable AI",
    "What is the Turing test?",
    "How does AI differ from traditional programming?",
    "What are the limitations of current AI systems?",
    "Explain the concept of general AI vs narrow AI",
    "What is the role of data in AI systems?",
    "How is AI used in recommendation systems?",
    "What are the future trends in AI?"
]


async def test_single_agent_query(query: str) -> Dict[str, Any]:
    """Test a single agent query and return results"""
    try:
        # Create a user query object
        user_query = UserQuery(query_text=query)

        # Initialize the agent
        agent = RAGAgent()

        # Process the query
        start_time = time.time()
        response = await agent.process_query(user_query)
        end_time = time.time()

        # Calculate metrics
        processing_time = end_time - start_time
        has_citations = len(response.source_citations) > 0
        confidence = response.confidence_score

        return {
            "query": query,
            "success": True,
            "processing_time": processing_time,
            "has_citations": has_citations,
            "confidence": confidence,
            "citations_count": len(response.source_citations),
            "response_length": len(response.content)
        }
    except Exception as e:
        return {
            "query": query,
            "success": False,
            "error": str(e),
            "processing_time": 0,
            "has_citations": False,
            "confidence": 0.0,
            "citations_count": 0,
            "response_length": 0
        }


async def run_comprehensive_agent_test():
    """Run comprehensive tests on the agent with 50 sample queries"""
    print("Starting comprehensive RAG Agent tests with 50 sample queries...")
    print("=" * 70)

    start_time = time.time()

    # Run tests for each query
    results = []
    successful_queries = 0
    total_citations = 0
    total_processing_time = 0
    total_confidence = 0

    for i, query in enumerate(SAMPLE_QUERIES, 1):
        result = await test_single_agent_query(query)
        results.append(result)

        if result["success"]:
            successful_queries += 1
            total_citations += result["citations_count"]
            total_processing_time += result["processing_time"]
            total_confidence += result["confidence"]

        # Print progress
        status = "✓" if result["success"] else "✗"
        print(f"{status} Query {i:2d}: {query[:50]}{'...' if len(query) > 50 else '':<20} "
              f"({result['processing_time']:.2f}s, {result['citations_count']} citations)")

    end_time = time.time()
    total_test_time = end_time - start_time

    # Calculate metrics
    success_rate = successful_queries / len(SAMPLE_QUERIES) * 100
    avg_processing_time = total_processing_time / len(SAMPLE_QUERIES) if len(SAMPLE_QUERIES) > 0 else 0
    avg_citations = total_citations / len(SAMPLE_QUERIES) if len(SAMPLE_QUERIES) > 0 else 0
    avg_confidence = total_confidence / successful_queries if successful_queries > 0 else 0

    # Print summary
    print("\n" + "=" * 70)
    print("TEST SUMMARY")
    print("=" * 70)
    print(f"Total Queries:           {len(SAMPLE_QUERIES)}")
    print(f"Successful Queries:      {successful_queries}")
    print(f"Failed Queries:          {len(SAMPLE_QUERIES) - successful_queries}")
    print(f"Success Rate:            {success_rate:.2f}%")
    print(f"Total Test Time:         {total_test_time:.2f}s")
    print(f"Average Processing Time: {avg_processing_time:.2f}s")
    print(f"Average Citations:       {avg_citations:.2f}")
    print(f"Average Confidence:      {avg_confidence:.2f}")

    # Success criteria checks
    print("\nSUCCESS CRITERIA CHECKS")
    print("-" * 30)

    # Check if success rate is above 99% threshold
    success_criteria_1 = success_rate >= 99
    print(f"✓ 99%+ Success Rate:     {'PASS' if success_criteria_1 else 'FAIL'} ({success_rate:.2f}%)")

    # Check if average processing time is under 5 seconds
    success_criteria_2 = avg_processing_time < 5.0
    print(f"✓ <5s Response Time:     {'PASS' if success_criteria_2 else 'FAIL'} ({avg_processing_time:.2f}s)")

    # Check if agent retrieves citations for factual queries
    success_criteria_3 = avg_citations > 0
    print(f"✓ Citations Retrieved:   {'PASS' if success_criteria_3 else 'FAIL'} ({avg_citations:.2f} avg)")

    # Overall success
    overall_success = all([success_criteria_1, success_criteria_2, success_criteria_3])
    print(f"\nOVERALL: {'PASS' if overall_success else 'FAIL'}")

    # Print any failed queries
    failed_queries = [r for r in results if not r["success"]]
    if failed_queries:
        print(f"\nFAILED QUERIES ({len(failed_queries)}):")
        for result in failed_queries:
            print(f"  - {result['query'][:60]}...: {result['error']}")

    return {
        "overall_success": overall_success,
        "success_rate": success_rate,
        "avg_processing_time": avg_processing_time,
        "avg_citations": avg_citations,
        "total_test_time": total_test_time,
        "results": results
    }


async def test_api_endpoint():
    """Test the API endpoint functionality"""
    print("\nTesting API endpoint...")

    try:
        # Create a test request
        request = AgentQueryRequest(
            query="What is artificial intelligence?",
            include_citations=True,
            max_chunks=5,
            similarity_threshold=0.7
        )

        # Call the API endpoint function directly
        start_time = time.time()
        response = await query_agent(request)
        end_time = time.time()

        processing_time = end_time - start_time

        print(f"✓ API endpoint test: SUCCESS ({processing_time:.2f}s)")
        print(f"  Response length: {len(response.response)} characters")
        print(f"  Citations: {response.retrieved_chunks_count}")
        print(f"  Confidence: {response.confidence:.2f}")

        return True

    except Exception as e:
        print(f"✗ API endpoint test: FAILED - {str(e)}")
        return False


async def main():
    """Main test function"""
    print("RAG Agent Construction - Comprehensive Testing Suite")
    print("=" * 70)

    # Test API endpoint first
    api_success = await test_api_endpoint()

    # Run comprehensive agent tests
    results = await run_comprehensive_agent_test()

    # Final summary
    print("\nFINAL SUMMARY")
    print("=" * 30)
    print(f"API Test:              {'PASS' if api_success else 'FAIL'}")
    print(f"Comprehensive Test:    {'PASS' if results['overall_success'] else 'FAIL'}")
    print(f"Overall Status:        {'SUCCESS' if api_success and results['overall_success'] else 'FAILURE'}")

    return results['overall_success'] and api_success


if __name__ == "__main__":
    success = asyncio.run(main())
    exit(0 if success else 1)