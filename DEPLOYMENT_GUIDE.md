# Deployment Guide for Vercel

## Current Issue
The website `https://hackathon-physical-ai-humanoid-text-beta.vercel.app/` was showing a 404 error because the project was configured for GitHub Pages deployment but not for Vercel.

## Solution Implemented
1. Created `vercel.json` configuration file to properly build and deploy the Docusaurus site.
2. Updated `website/docusaurus.config.js` to use the correct URL and base path for Vercel deployment.
3. Added root-level `package.json` with build scripts for Vercel.

## Files Modified
- `vercel.json`: Added Vercel build configuration.
- `website/docusaurus.config.js`: Updated URL to `https://hackathon-physical-ai-humanoid-text-beta.vercel.app` and baseUrl to `/`.
- `package.json` (root): Added build and start scripts for Vercel deployment.

## Steps to Deploy to Vercel

### Option 1: Using Vercel CLI
1. Install Vercel CLI: `npm install -g vercel`
2. Navigate to your project root: `cd D:\Python\Open_ai\hackathon-physical-ai-humanoid-textbook`
3. Run: `vercel --prod` (for production deployment)

### Option 2: Connect GitHub Repository to Vercel Dashboard
1. Push all changes to your GitHub repository.
2. Go to https://vercel.com/dashboard
3. Click "Add New Project" and import your GitHub repository.
4. Vercel will automatically detect the Docusaurus project and use the `vercel.json` configuration.
5. The project will build and deploy automatically.

### Option 3: Manual Deployment
1. Build the site locally: `cd website && npm run build`
2. The build output will be in `website/build/`
3. Upload this to Vercel using their dashboard or CLI.

## Configuration Details
- The site is now configured to deploy at the root path (`/`) rather than a subdirectory.
- Vercel will run `npm run build` from the root directory, which will install dependencies in the website folder and build the site.
- The output in `website/build/` will be served as the static site.

## Important Notes
- If you want to continue using GitHub Pages alongside Vercel, you may need to maintain separate configurations.
- For GitHub Pages deployment, you would need to switch back the `baseUrl` to `/Robotic-ai-Book/` and update the URL.
- The current configuration is optimized for Vercel deployment at the root domain.

