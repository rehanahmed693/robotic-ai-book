# API Configuration Documentation

## Base URL
The API base URL is configured via the environment variable `REACT_APP_API_BASE_URL`.
Default value: `http://127.0.0.1:8000`

To set a custom API URL:
1. Create or edit the `.env` file in the `my-website` directory
2. Add the following line:
   ```
   REACT_APP_API_BASE_URL=https://your-api-url.com
   ```

## Available Endpoints

### Chat Session Management
- `POST /api/v1/chat/start` - Start a new chat session
- `POST /api/v1/chat/end` - End a chat session
- `GET /api/v1/chat/history` - Get conversation history for a session

### Query Processing
- `POST /api/v1/chat/query` - Submit a query to the chatbot

## Timeout Configuration
- Default request timeout: 10 seconds
- Requests exceeding this timeout will be cancelled and result in an error

## Error Handling
The API service includes comprehensive error handling for:
- Network errors
- HTTP error responses (4xx, 5xx)
- Timeout errors
- CORS-related errors

## Connection Logging
Connection events are logged for debugging purposes:
- Request initiation and completion
- Error events with details
- Connection status changes

The logs can be accessed through the ConnectionLogger service.