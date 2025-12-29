import requests
import json

def test_guardrails(query):
    url = "http://localhost:8000/v1/agent/query"
    payload = {"query": query}
    try:
        response = requests.post(url, json=payload)
        data = response.json()
        print(f"Query: {query}")
        print(f"Agent: {data['response']}")
        print("-" * 30)
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    print("Testing Agent Guardrails...\n")
    
    # Valid technical query
    test_guardrails("What is embodied AI?")
    
    # Friendly greeting
    test_guardrails("Hello!")
    
    # Out of scope query (Cooking)
    test_guardrails("How to make Biryani?")
    
    # Out of scope query (Politics)
    test_guardrails("Who is the prime minister of Pakistan?")
