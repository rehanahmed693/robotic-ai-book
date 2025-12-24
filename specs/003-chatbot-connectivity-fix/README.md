# RAG Chatbot Frontend Connectivity Fix

## Overview
This document describes the changes made to fix the "Failed to connect to the chatbot service" error in the RAG chatbot frontend.

## Changes Made

### 1. API Service Module
- Created a centralized API service module (`my-website/src/services/api.ts`) to handle all communication with the backend
- Added proper error handling, timeout mechanisms, and connection retry logic

### 2. Environment Configuration
- Configured API base URL through environment variables in `.env` file
- Default URL is set to `http://127.0.0.1:8000` to match the backend service

### 3. Enhanced Error Handling
- Implemented comprehensive error handling in `my-website/src/services/errorHandler.ts`
- Added specific error messages for different types of failures (network, timeout, server errors)
- Improved error messages displayed to users

### 4. Connection Stability
- Added timeout mechanisms (10 seconds) to prevent hanging requests
- Implemented automatic reconnection logic in the ChatBot component
- Added connection logging for debugging purposes

### 5. ChatBot UI Improvements
- Updated the ChatBot component to use the new API service
- Added retry button for manual connection attempts
- Improved error display with user-friendly messages

## API Endpoints Used
- `POST /api/v1/chat/start` - Start a new chat session
- `POST /api/v1/chat/query` - Submit a query to the chatbot
- `POST /api/v1/chat/end` - End a chat session
- `GET /api/v1/chat/history` - Get conversation history

## Testing
A connectivity test is available in `my-website/src/services/connectivityTest.ts` that verifies:
- Session creation
- Query submission
- Session termination

## Configuration
To customize the backend API URL, update the `.env` file in the `my-website` directory:
```
REACT_APP_API_BASE_URL=https://your-backend-url.com
```

## Troubleshooting
If the connection issue persists:
1. Verify that the backend service is running on the configured port
2. Check the browser console for specific error messages
3. Review the connection logs using the ConnectionLogger service