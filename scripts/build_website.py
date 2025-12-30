"""Helper script to build the Docusaurus website from Python.

Usage:
    python scripts/build_website.py

This runs `npm ci` and `npm run build` in the `website` folder.
"""
import os
import subprocess
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
WEBSITE_DIR = os.path.join(ROOT_DIR, "website")

if not os.path.isdir(WEBSITE_DIR):
    print("website folder not found", file=sys.stderr)
    sys.exit(1)

try:
    print("Running `npm ci` in website/ ...")
    subprocess.check_call(["npm", "ci"], cwd=WEBSITE_DIR)

    print("Running `npm run build` in website/ ...")
    subprocess.check_call(["npm", "run", "build"], cwd=WEBSITE_DIR)

    print("Build completed. The built site is available in website/build/")
except subprocess.CalledProcessError as e:
    print(f"Build failed: {e}", file=sys.stderr)
    sys.exit(2)
