"""Root FastAPI app that mounts the backend and serves the Docusaurus build.

- Re-uses the existing `backend.main.app` FastAPI app which already registers the agent/chat routes.
- Adds CORS middleware so the frontend can call the API from the same origin or other dev servers.
- Mounts the Docusaurus `website/build` output at the site root when present.
- Provides a helpful message when the build is missing and an optional endpoint to trigger a build.

Run with:
    python api.py
or
    uvicorn api:app --reload --host 0.0.0.0 --port 8000
"""
import os
import subprocess
import logging
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from starlette.staticfiles import StaticFiles

# Import the existing app (this will include routers and lifespan from backend/main.py)
# Ensure backend/ is on sys.path so absolute imports like `src.*` inside the backend package resolve
import sys
ROOT_DIR = os.path.dirname(__file__)
BACKEND_DIR = os.path.join(ROOT_DIR, "backend")
if BACKEND_DIR not in sys.path:
    sys.path.insert(0, BACKEND_DIR)

try:
    from backend.main import app as app  # reuse existing application
except Exception as e:
    # If importing the app fails, create a minimal FastAPI app to at least serve the site
    logging.warning(f"Failed to import backend.main.app, creating a fresh FastAPI app: {e}")
    app = FastAPI(title="Physical AI API (fallback)")

# Add permissive CORS for local development; adjust origins for production as needed
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

ROOT_DIR = os.path.dirname(__file__)
WEBSITE_BUILD_DIR = os.path.join(ROOT_DIR, "website", "build")

if os.path.isdir(WEBSITE_BUILD_DIR):
    # Mount static files at root (place after API routers are included so API routes stay live)
    app.mount("/", StaticFiles(directory=WEBSITE_BUILD_DIR, html=True), name="website")
else:
    @app.get("/")
    async def index_not_built():
        return {
            "message": "Website build not found. Run `cd website && npm install && npm run build` to generate the build.",
            "build_path": WEBSITE_BUILD_DIR,
        }

# Optional build endpoint (disabled by default). To enable set env var ENABLE_SITE_BUILD_ENDPOINT=1
ENABLE_BUILD_ENDPOINT = os.getenv("ENABLE_SITE_BUILD_ENDPOINT", "0") == "1"

if ENABLE_BUILD_ENDPOINT:
    @app.post("/admin/build-site")
    async def build_site():
        """Trigger `npm run build` in the website folder. Use only for local/dev environments."""
        if not os.path.isdir(os.path.join(ROOT_DIR, "website")):
            raise HTTPException(status_code=400, detail="Website folder not found")

        try:
            # Run npm install then npm run build
            subprocess.check_call(["npm", "ci"], cwd=os.path.join(ROOT_DIR, "website"))
            subprocess.check_call(["npm", "run", "build"], cwd=os.path.join(ROOT_DIR, "website"))
            return {"status": "built", "path": WEBSITE_BUILD_DIR}
        except subprocess.CalledProcessError as err:
            raise HTTPException(status_code=500, detail=f"Build failed: {err}")


# Uvicorn run for convenience
if __name__ == "__main__":
    import uvicorn
    uvicorn.run("api:app", host="0.0.0.0", port=8000, reload=True)
