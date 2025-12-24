# Quickstart Guide: Integrated RAG Chatbot for a Published Book

## Setup and Installation

### Prerequisites
- Python 3.11+
- Qdrant instance (can be local or cloud)
- Neon Postgres database
- OpenAI or Cohere API key

### Environment Configuration
Create a `.env` file with the following variables:
```bash
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_database_url
OPENAI_API_KEY=your_openai_api_key  # or COHERE_API_KEY
```

### Installation
1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

2. Set up the database schema:
```bash
# Run database migrations
python -m src.core.database.migrate
```

3. Index your book content:
```bash
python -m src.core.book_indexer --book-path path/to/your/book.txt
```

## Usage

### 1. Starting a Session
```bash
curl -X POST http://localhost:8000/api/v1/chat/start \
  -H "Content-Type: application/json" \
  -d '{
    "book_id": "your-book-uuid"
  }'
```

### 2. Submitting Queries
```bash
# Global mode (default)
curl -X POST http://localhost:8000/api/v1/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "your-session-uuid",
    "question": "What is the main theme of this book?"
  }'

# Selection-only mode
curl -X POST http://localhost:8000/api/v1/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "your-session-uuid",
    "question": "What does this selected text mean?",
    "rag_mode": "selection_only",
    "selection_context": "The text the user highlighted..."
  }'
```

### 3. Getting Conversation History
```bash
curl -X GET "http://localhost:8000/api/v1/chat/history?session_id=your-session-uuid"
```

### 4. Ending a Session
```bash
curl -X POST http://localhost:8000/api/v1/chat/end \
  -H "Content-Type: application/json" \
  -d '{
    "session_id": "your-session-uuid"
  }'
```

## Running the Server

### Development
```bash
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

### Production
```bash
gunicorn src.api.main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:8000
```

## Key Components

- `src/api/main.py`: FastAPI application entry point
- `src/services/rag_service.py`: Core RAG logic
- `src/services/chat_service.py`: Chat session management
- `src/vector_store/qdrant_client.py`: Vector database interactions
- `src/database/`: Database models and operations
- `src/models/`: Data models and schemas