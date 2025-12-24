# Data Model: Integrated RAG Chatbot for a Published Book

## Entities

### Book Content
- **book_id** (UUID): Unique identifier for the book
- **title** (string): Title of the book
- **author** (string): Author of the book
- **content** (text): Full text content of the book
- **created_at** (timestamp): When the book was added to the system
- **updated_at** (timestamp): When the book content was last updated

### Chunk
- **chunk_id** (UUID): Unique identifier for the chunk
- **book_id** (UUID): Reference to the parent book
- **content** (text): Text content of the chunk
- **chunk_index** (integer): Sequential position of the chunk in the book
- **metadata** (JSON): Additional information like page number, section, etc.
- **vector_id** (string): ID in the vector database (Qdrant)

### Session
- **session_id** (UUID): Unique identifier for the conversation session
- **user_id** (UUID, optional): Reference to user (if applicable)
- **book_id** (UUID): Reference to the book being queried
- **created_at** (timestamp): When the session started
- **updated_at** (timestamp): When the session was last active
- **expires_at** (timestamp): When the session expires

### Conversation
- **conversation_id** (UUID): Unique identifier for the conversation entry
- **session_id** (UUID): Reference to the session
- **role** (string): "user" or "assistant"
- **content** (text): The message content
- **timestamp** (timestamp): When the message was created
- **citations** (JSON): List of citations for assistant responses

### Query
- **query_id** (UUID): Unique identifier for the query
- **session_id** (UUID): Reference to the session
- **question** (text): The user's question
- **rag_mode** (string): "global" or "selection_only"
- **selection_context** (text, optional): User-selected text for selection-only mode
- **timestamp** (timestamp): When the query was made

### Answer
- **answer_id** (UUID): Unique identifier for the answer
- **query_id** (UUID): Reference to the query
- **response** (text): The generated answer
- **citations** (JSON): Citations to book content that support the response
- **confidence_score** (float): Confidence level of the response
- **timestamp** (timestamp): When the answer was generated

### VectorMetadata
- **vector_id** (string): Reference to the vector in Qdrant
- **book_id** (UUID): Reference to the book
- **chunk_id** (UUID): Reference to the chunk
- **metadata** (JSON): Additional metadata for retrieval

## Relationships

- Book Content (1) → Chunk (Many): A book is divided into multiple chunks
- Book Content (1) → Session (Many): A book can be queried in multiple sessions
- Session (1) → Conversation (Many): A session contains multiple conversation entries
- Session (1) → Query (Many): A session contains multiple queries
- Query (1) → Answer (1): A query produces one answer
- Answer → Chunk (Many): An answer can cite multiple chunks

## Validation Rules

- Book content must be non-empty when creating a book entry
- Chunk content must be non-empty and within token limits
- Session must be associated with a valid book
- Query must specify a valid RAG mode
- Answer citations must reference valid chunks from the same book
- Selection context in selection-only mode must be non-empty

## State Transitions (if applicable)

- Session: active → expired (based on inactivity timer)
- Query: pending → processed → answered