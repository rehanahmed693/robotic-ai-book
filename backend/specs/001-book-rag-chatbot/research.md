# Research: Integrated RAG Chatbot for a Published Book

## Key Decisions and Rationale

### Decision: Vector database choice (Qdrant vs alternatives)
**Rationale**: Qdrant was specified in the original requirements and is well-suited for similarity search in RAG applications. It offers good performance and the free tier meets our initial requirements.
**Alternatives considered**: 
- Pinecone: Commercial solution with good features but might exceed budget
- Weaviate: Open source alternative with good capabilities but slightly less familiarity in the team
- Elasticsearch: Good for text search but not as specialized for vector search as Qdrant

### Decision: Embedding model choice (OpenAI vs Cohere)
**Rationale**: OpenAI embeddings were specified in the original requirements and are known for high quality, though Cohere might provide better performance for domain-specific content at potentially lower cost.
**Alternatives considered**:
- Cohere: Potentially better for book content, cost-effective, good performance
- OpenAI: High quality embeddings, well-documented, familiar to many developers

### Decision: Chunking strategy
**Rationale**: For book content, we'll use semantic chunking rather than fixed-size chunking to maintain context integrity while ensuring chunks are small enough for model limitations.
**Alternatives considered**:
- Fixed-size: 512/1024 tokens - simpler but may break up context
- Semantic: Break at natural boundaries like paragraphs/sentences - maintains context but more complex

### Decision: RAG mode separation
**Rationale**: Implement unified pipeline with mode switching to reduce code duplication while maintaining clear separation of concerns between global and selection-only modes.
**Alternatives considered**:
- Isolated pipelines: Separate code paths for each mode - clearer separation but more maintenance
- Unified pipeline: Single pipeline with mode selection - less duplication but requires careful design

### Decision: Conversation memory storage
**Rationale**: Store conversation history in the relational database (Neon Postgres) for persistence and consistency across server restarts, but cache active conversations in memory for better performance.
**Alternatives considered**:
- Database only: Persistent but slower for active conversations
- In-memory only: Fast but loses state on server restarts
- Hybrid approach: Cache active conversations in memory, persist to DB periodically

### Decision: Citation method
**Rationale**: Use metadata-based citations that reference specific passages in the book content, allowing for precise attribution without cluttering the user-facing response.
**Alternatives considered**:
- Inline citations: Direct quotes in the response - clear but may clutter
- Metadata-based: Citations as reference links - cleaner UI but requires post-processing

## Technical Research Findings

### RAG Accuracy and Performance
Research confirms that semantic chunking with appropriate overlap (100-150 tokens) between chunks helps maintain context while ensuring relevant information retrieval. The quality of embeddings significantly impacts retrieval accuracy.

### Citation Enforcement
To enforce citation accuracy, the system will track which source documents/passages contributed to each response, enabling verification that all claims are supported by the retrieved context.

### Hallucination Prevention
Key strategies for preventing hallucinations, especially in selection-only mode:
1. Strict context filtering - Only allow the selected text as context
2. Confidence scoring - Indicate when the system is uncertain
3. Grounding checks - Verify that responses are based on provided context
4. Explicit mode switching - Clear boundaries between global and selection modes

### Quality Validation Methods
For both global and selection-only modes:
- Manual review of responses against source content
- Automated checks for citation accuracy
- Tracking metrics like relevance and factual correctness
- A/B testing different approaches with users

## Implementation Considerations

### Data Flow Design
- Book content → preprocessing → semantic chunking → embeddings → Qdrant storage
- User query → embedding → similarity search → context retrieval → response generation
- Conversation history → persistence in Neon Postgres → session management

### API Design Patterns
The API will follow REST principles with endpoints for:
- Query processing (with mode selection)
- Session management
- Book content management
- Citation retrieval

### Security and Validation
- Input sanitization to prevent injection attacks
- Rate limiting to prevent abuse
- Proper authentication if required for multi-user scenarios
- Environment variable-based configuration for secrets