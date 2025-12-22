import os
import httpx
import asyncio

async def test_openrouter():
    api_key = "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"

    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json"
    }

    # Test chat completion
    client = httpx.AsyncClient(timeout=30.0)

    try:
        # Test chat completion
        payload = {
            "model": "openai/gpt-4o",
            "messages": [
                {"role": "user", "content": "Hello, just say 'Test successful'"}
            ],
            "max_tokens": 20
        }

        response = await client.post(
            "https://openrouter.ai/api/v1/chat/completions",
            json=payload,
            headers=headers
        )
        response.raise_for_status()

        result = response.json()
        print(f"Chat completion successful: {result['choices'][0]['message']['content']}")

    except Exception as e:
        print(f"Chat completion failed: {e}")

    # Test embedding model availability
    try:
        payload = {
            "model": "thenlper/gte-large-en-v1.5",
            "input": ["test"]
        }

        response = await client.post(
            "https://openrouter.ai/api/v1/embeddings",
            json=payload,
            headers=headers
        )
        response.raise_for_status()

        result = response.json()
        print(f"Embedding model works: {len(result['data'][0]['embedding'])} dimensions")

    except Exception as e:
        print(f"Embedding test failed: {e}")

    await client.aclose()

if __name__ == "__main__":
    asyncio.run(test_openrouter())