"""Simple test script to check chat endpoint"""
import requests
import json

def test_chat():
    url = "http://localhost:8000/v1/chat/query"
    
    payload = {
        "query": "What is Physical AI?",
        "session_id": "test_session",
        "include_citations": True
    }
    
    print("Sending request to:", url)
    print("Payload:", json.dumps(payload, indent=2))
    
    try:
        response = requests.post(
            url, 
            json=payload,
            timeout=30  # 30 second timeout
        )
        
        print(f"\nStatus Code: {response.status_code}")
        print(f"Response: {response.text}")
        
        if response.status_code == 200:
            data = response.json()
            print("\n✅ Success!")
            print(f"Response: {data.get('response', 'No response')[:200]}...")
        else:
            print(f"\n❌ Error: {response.status_code}")
            
    except requests.exceptions.Timeout:
        print("\n❌ Request timed out after 30 seconds!")
        print("This usually means the backend is stuck processing the request.")
    except requests.exceptions.ConnectionError:
        print("\n❌ Could not connect to backend!")
        print("Make sure the backend is running on port 8000")
    except Exception as e:
        print(f"\n❌ Error: {str(e)}")

if __name__ == "__main__":
    test_chat()
