import argparse
import asyncio
import sys
import os
from typing import List, Dict, Any
import logging
import json

# Add the backend directory to Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from dotenv import load_dotenv
from ..validation.retrieval_validator import RetrievalValidator
from ..config.validation_config import validation_config


class ValidationCLI:
    """
    Command-line interface for the RAG validation pipeline
    """

    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        load_dotenv()  # Load environment variables

        # Initialize validation service
        self.validator = RetrievalValidator(validation_config)

    async def run_validation(self, queries: List[str], validation_name: str = "cli_validation") -> Dict[str, Any]:
        """
        Run validation on the provided queries
        """
        self.logger.info(f"Starting CLI validation: {validation_name} with {len(queries)} queries")

        try:
            # Run the validation pipeline
            report = await self.validator.run_validation(queries, validation_name)

            result = {
                "validation_name": validation_name,
                "total_queries": report.total_queries,
                "successful_retrievals": report.successful_retrievals,
                "precision_score": report.precision_score,
                "average_latency": report.average_latency,
                "metadata_accuracy": report.metadata_accuracy,
                "determinism_score": report.determinism_score,
                "status": report.status.value
            }

            self.logger.info(f"CLI validation completed: {result}")
            return result

        except Exception as e:
            self.logger.error(f"Error in CLI validation: {str(e)}")
            raise

    def run(self):
        """
        Run the CLI with command-line arguments
        """
        parser = argparse.ArgumentParser(description="RAG Retrieval Validation CLI")
        parser.add_argument("--queries", nargs="+", required=True, help="Queries to validate")
        parser.add_argument("--validation-name", default="cli_validation", help="Name for the validation run")
        parser.add_argument("--top-k", type=int, default=5, help="Number of results to retrieve for each query")
        parser.add_argument("--threshold", type=float, default=0.7, help="Similarity threshold for relevance")
        parser.add_argument("--output", default="json", choices=["json", "text"], help="Output format")

        args = parser.parse_args()

        # Set up logging
        logging.basicConfig(level=logging.INFO)

        # Update config with command-line parameters
        validation_config.validation_top_k = args.top_k
        validation_config.validation_threshold = args.threshold

        # Run the validation pipeline
        result = asyncio.run(self.run_validation(args.queries, args.validation_name))

        # Print results based on output format
        if args.output == "json":
            print(json.dumps(result, indent=2))
        else:  # text format
            print(f"\nValidation Results:")
            print(f"  Validation Name: {result['validation_name']}")
            print(f"  Total Queries: {result['total_queries']}")
            print(f"  Successful Retrievals: {result['successful_retrievals']}")
            print(f"  Precision Score: {result['precision_score']:.2f}")
            print(f"  Average Latency: {result['average_latency']:.2f}s")
            print(f"  Metadata Accuracy: {result['metadata_accuracy']:.2f}")
            print(f"  Determinism Score: {result['determinism_score']:.2f}")
            print(f"  Status: {result['status']}")

        return result


def main():
    """
    Main function to run the CLI
    """
    cli = ValidationCLI()
    return cli.run()


if __name__ == "__main__":
    main()