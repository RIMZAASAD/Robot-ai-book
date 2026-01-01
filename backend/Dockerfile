# Physical AI & Humanoid Robotics Backend Dockerfile
# ================================================
# This Dockerfile creates a container for the backend service of the Physical AI & Humanoid Robotics textbook platform
#
# Key Backend Components:
# - main.py: FastAPI application entry point
# - run_server_port8001.py: Server startup script
# - src/: Contains core backend services and models
# - services/: Chat, embedding, and vector database services
# - models/: Pydantic data models
# - requirements.txt: Python dependencies including FastAPI, Anthropic, Qdrant, etc.
# - .env.example: Environment configuration template
#
# Features:
# - AI-powered chatbot with RAG (Retrieval Augmented Generation)
# - Vector database integration with Qdrant
# - Anthropic Claude API integration
# - PDF processing and content extraction
# - Semantic search capabilities

FROM python:3.11-slim

RUN useradd -m -u 1000 user
USER user
ENV PATH="/home/user/.local/bin:$PATH"

WORKDIR /app

COPY --chown=user ./requirements.txt requirements.txt
RUN pip install --no-cache-dir --upgrade -r requirements.txt

COPY --chown=user . /app
EXPOSE 8001

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8001"]