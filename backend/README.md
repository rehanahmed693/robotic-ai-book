# RAG Chatbot Backend

This is the backend for the RAG (Retrieval Augmented Generation) chatbot that answers questions about book content.

## Prerequisites

- Python 3.9 or higher
- PostgreSQL database (e.g., Neon, AWS RDS, or local installation)
- Qdrant vector store (cloud or local)
- API keys for language models (OpenAI, Cohere, or Google)

## Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd <repository-directory>/backend
   ```

2. Create and activate a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

4. Copy the example environment file and update with your actual configuration:
   ```bash
   cp .env.example .env
   ```
   
   Edit the `.env` file with your actual API keys and database connection strings.

5. Run the application:
   ```bash
   uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
   ```

## Environment Variables

The application requires the following environment variables:

- `NEON_DATABASE_URL`: PostgreSQL database connection string
- `QDRANT_URL`: URL for the Qdrant vector store
- `QDRANT_API_KEY`: API key for Qdrant (if required)
- `OPENAI_API_KEY`, `COHERE_API_KEY`, or `GOOGLE_API_KEY`: API key for your chosen LLM provider
- `DEBUG`: Set to `true` for development

## API Endpoints

- `GET /` - Root endpoint
- `GET /health` - Health check
- `POST /api/v1/chat/start` - Start a new chat session
- `POST /api/v1/chat/message` - Send a message and get a response
- `POST /api/v1/chat/query` - Submit a query with session management
- `POST /api/v1/chat/end` - End a chat session
- `GET /api/v1/chat/history` - Get conversation history

## Running with Docker (Optional)

If you prefer to run with Docker:

1. Build the image:
   ```bash
   docker build -t rag-chatbot-backend .
   ```

2. Run the container:
   ```bash
   docker run -p 8000:8000 --env-file .env rag-chatbot-backend
   ```

## Troubleshooting

1. If you see "WARNING: No database URL provided", ensure your `.env` file has the correct `NEON_DATABASE_URL` variable set.
2. If you get SSL connection errors, ensure your PostgreSQL URL includes the correct SSL parameters.
3. If you encounter 500 errors, check the server logs for detailed error messages.
4. If sessions are not found, ensure the session management logic is working properly and sessions aren't expiring too quickly.

## Development

To run tests:
```bash
pytest
```

To run with auto-reload during development:
```bash
uvicorn src.main:app --reload
```