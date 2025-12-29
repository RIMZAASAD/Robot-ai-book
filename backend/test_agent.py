import requests
import json

def test_agent(query):
    url = "http://localhost:8000/v1/agent/query"
    payload = {
        "query": query,
        "include_citations": True
    }
    print(f"\nSending Query: {query}")
    try:
        response = requests.post(url, json=payload)
        if response.status_code == 200:
            data = response.json()
            print("Response Received Successfully!")
            print("-" * 30)
            print(f"AI Answer: {data['response']}")
            print(f"Citations: {len(data['citations'])}")
            print(f"Confidence: {data['confidence']:.2f}")
        else:
            print(f"Error: {response.status_code}")
            print(response.text)
    except Exception as e:
        print(f"Connection Error: {e}")

if __name__ == "__main__":
    # Test a factual query (should trigger retrieval)
    test_agent("What is Physical AI?")
    
    # Test a greeting (should skip retrieval - US2)
    test_agent("Hello! How are you today?")
