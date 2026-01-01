# Multi-stage Dockerfile for Physical AI & Humanoid Robotics Textbook

# Stage 1: Build the Docusaurus frontend
FROM node:18-alpine AS frontend-builder

WORKDIR /app

# Copy package files
COPY website/package*.json ./website/

# Install dependencies
WORKDIR /app/website
RUN npm ci --only=production

# Copy source code
COPY website/ ./

# Build the Docusaurus site
RUN npm run build

# Stage 2: Set up the Python backend
FROM python:3.11-slim AS backend

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    && rm -rf /var/lib/apt/lists/*

# Copy Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# Copy backend source code
COPY src/ ./src/
COPY run_server_port8001.py ./
COPY .env.example ./

# Copy built frontend from previous stage
COPY --from=frontend-builder /app/website/build /app/website/build

# Expose port
EXPOSE 8001

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:8001/health || exit 1

# Command to run the application
CMD ["python", "run_server_port8001.py"]