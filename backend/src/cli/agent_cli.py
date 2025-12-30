"""
CLI interface for the RAG Agent Construction feature.

This module provides command-line interface for testing and interacting
with the RAG agent.
"""

import asyncio
import argparse
import sys
import logging
from typing import List
from ..models.agent_models import UserQuery
from ..agents.rag_agent import RAGAgent


def setup_logging():
    """Setup logging for the CLI interface"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )


async def run_single_query(query_text: str, include_citations: bool = True, max_chunks: int = 5):
    """Run a single query against the RAG agent"""
    setup_logging()
    logger = logging.getLogger(__name__)

    logger.info(f"Processing query: {query_text}")

    try:
        # Initialize the RAG agent
        agent = RAGAgent()

        # Create a user query object
        user_query = UserQuery(
            query_text=query_text,
            metadata={
                "include_citations": include_citations,
                "max_chunks": max_chunks
            }
        )

        # Process the query
        response = await agent.process_query(user_query)

        # Print the response
        print(f"\nQuery: {query_text}")
        print(f"Response: {response.content}")
        print(f"Confidence: {response.confidence_score:.2f}")

        if response.source_citations:
            print(f"\nCitations ({len(response.source_citations)}):")
            for i, citation in enumerate(response.source_citations, 1):
                print(f"  {i}. Source: {citation.get('source_url', 'N/A')}")
                print(f"     Document ID: {citation.get('document_id', 'N/A')}")
                print(f"     Similarity: {citation.get('similarity_score', 0):.2f}")
                print(f"     Preview: {citation.get('content_preview', '')[:100]}...")
                print()

    except Exception as e:
        logger.error(f"Error processing query: {str(e)}")
        print(f"Error: {str(e)}")
        return False

    return True


async def run_batch_queries(queries: List[str], output_format: str = "text"):
    """Run multiple queries against the RAG agent"""
    setup_logging()
    logger = logging.getLogger(__name__)

    logger.info(f"Processing {len(queries)} queries in batch")

    # Initialize the RAG agent
    agent = RAGAgent()
    results = []

    for i, query_text in enumerate(queries, 1):
        logger.info(f"Processing query {i}/{len(queries)}: {query_text[:50]}...")

        try:
            # Create a user query object
            user_query = UserQuery(query_text=query_text)

            # Process the query
            response = await agent.process_query(user_query)

            result = {
                "query": query_text,
                "response": response.content,
                "confidence": response.confidence_score,
                "citations_count": len(response.source_citations)
            }
            results.append(result)

            if output_format == "text":
                print(f"\n--- Query {i} ---")
                print(f"Q: {query_text}")
                print(f"A: {response.content}")
                print(f"Conf: {response.confidence_score:.2f}")
                print(f"Citations: {len(response.source_citations)}")

        except Exception as e:
            logger.error(f"Error processing query {i}: {str(e)}")
            result = {
                "query": query_text,
                "error": str(e),
                "success": False
            }
            results.append(result)

    if output_format == "json":
        import json
        print(json.dumps(results, indent=2))

    return results


def main():
    """Main CLI entry point"""
    parser = argparse.ArgumentParser(
        description="CLI for testing the RAG Agent Construction feature"
    )

    # Add subcommands
    subparsers = parser.add_subparsers(dest="command", help="Available commands")

    # Single query command
    query_parser = subparsers.add_parser("query", help="Run a single query")
    query_parser.add_argument("text", help="The query text to process")
    query_parser.add_argument("--no-citations", action="store_true",
                             help="Don't include citations in response")
    query_parser.add_argument("--max-chunks", type=int, default=5,
                             help="Maximum number of chunks to retrieve (default: 5)")

    # Batch query command
    batch_parser = subparsers.add_parser("batch", help="Run multiple queries")
    batch_parser.add_argument("queries", nargs="+", help="Queries to process")
    batch_parser.add_argument("--format", choices=["text", "json"], default="text",
                             help="Output format (default: text)")

    # Test command
    test_parser = subparsers.add_parser("test", help="Run agent tests")
    test_parser.add_argument("--queries-file", help="File containing test queries (one per line)")
    test_parser.add_argument("--count", type=int, default=5,
                            help="Number of sample queries to run (default: 5)")

    args = parser.parse_args()

    if args.command == "query":
        success = asyncio.run(
            run_single_query(
                args.text,
                include_citations=not args.no_citations,
                max_chunks=args.max_chunks
            )
        )
        sys.exit(0 if success else 1)

    elif args.command == "batch":
        results = asyncio.run(
            run_batch_queries(args.queries, output_format=args.format)
        )
        sys.exit(0)

    elif args.command == "test":
        if args.queries_file:
            # Read queries from file
            with open(args.queries_file, 'r', encoding='utf-8') as f:
                queries = [line.strip() for line in f if line.strip()]
        else:
            # Use sample queries
            queries = [
                "What is artificial intelligence?",
                "Explain machine learning concepts",
                "How does natural language processing work?",
                "What are neural networks?",
                "Describe computer vision applications"
            ]
            # Limit to specified count
            queries = queries[:args.count]

        print(f"Running {len(queries)} test queries...")
        results = asyncio.run(
            run_batch_queries(queries, output_format="text")
        )
        print(f"\nCompleted {len(results)} test queries")
        sys.exit(0)

    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()