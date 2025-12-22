import asyncio
import httpx
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

async def ingest_textbook():
    async with httpx.AsyncClient() as client:
        try:
            print("Starting textbook ingestion...")
            response = await client.post(
                "http://localhost:8000/v1/ingest-textbook",
                timeout=300  # 5 minute timeout for potentially large ingestion
            )

            print(f"Response status: {response.status_code}")
            print(f"Response: {response.text}")

            if response.status_code == 200:
                print("[SUCCESS] Textbook ingestion completed successfully!")
            else:
                print(f"[ERROR] Textbook ingestion failed with status: {response.status_code}")

        except httpx.ConnectError:
            print("[ERROR] Cannot connect to the backend server. Make sure it's running on http://localhost:8000")
        except Exception as e:
            print(f"[ERROR] Error during ingestion: {e}")

if __name__ == "__main__":
    asyncio.run(ingest_textbook())