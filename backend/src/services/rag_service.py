from typing import List, Optional, Dict, Any
from uuid import UUID, uuid4
import logging
import asyncio
from ..models.query import Query, QueryCreate, RAGMode
from ..models.answer import Answer, AnswerCreate, Citation
from ..models.chunk import Chunk
from ..core.config import settings
from ..vector_store.qdrant_client import qdrant_client
from ..database.connections import db_connection
from ..services.session_service import SessionService
from ..core.embeddings import get_embeddings
import cohere
import tiktoken


class RAGService:
    def __init__(self, session_service=None):
        self.session_service = session_service or SessionService()
        # Initialize Cohere client
        if settings.cohere_api_key:
            self.co = cohere.AsyncClient(settings.cohere_api_key)
        self.enc = tiktoken.encoding_for_model("gpt-3.5-turbo")

    async def process_query(self, query: QueryCreate) -> Answer:
        """Process a user query and return an answer with citations"""
        try:
            # Generate a query_id for the new query since QueryCreate doesn't have one
            query_id = uuid4()

            # Get the session to retrieve the book ID
            session = await self.session_service.get_session(query.session_id)
            if not session:
                logging.warning(f"Session {query.session_id} not found")
                # Return a response indicating that a valid session is required
                return Answer(
                    query_id=query_id,
                    response="Please start a valid session before asking questions.",
                    citations=[],
                    confidence_score=0.0,
                    answer_id=UUID(int=0)  # Placeholder ID
                )

            book_id = session.book_id

            # Search for relevant chunks in Qdrant
            if query.rag_mode == RAGMode.SELECTION_ONLY and query.selection_context:
                # For selection-only mode, use the provided selection context
                relevant_chunks = await self._create_selection_chunks(query.selection_context)
            elif book_id is None:
                # If no book_id is available, we can't search in vector store
                # Return a response indicating that a book needs to be selected
                response_text, citations = await self._generate_answer_with_context(
                    query.question,
                    [],
                    query.rag_mode
                )
            else:
                # Get embeddings for the query using Cohere
                try:
                    query_embedding = await get_embeddings(query.question, use_cohere=True)
                except Exception as e:
                    logging.error(f"Error generating embeddings for query: {e}")
                    return Answer(
                        query_id=query_id,
                        response="I'm having trouble understanding your question. Please try rephrasing.",
                        citations=[],
                        confidence_score=0.0,
                        answer_id=UUID(int=0)  # Placeholder ID
                    )

                # For global mode, search the entire book
                try:
                    search_results = await qdrant_client.search_vectors(
                        query_vector=query_embedding,
                        limit=10,
                        book_id=str(book_id)
                    )
                except Exception as e:
                    logging.error(f"Error searching in Qdrant: {e}")
                    # Check if the book has any embeddings at all
                    try:
                        # Count points for this book to see if embeddings exist
                        from qdrant_client.http import models
                        count = qdrant_client.client.count(
                            collection_name=qdrant_client.collection_name,
                            count_filter=models.Filter(
                                must=[
                                    models.FieldCondition(
                                        key="book_id",
                                        match=models.MatchValue(value=str(book_id))
                                    )
                                ]
                            )
                        )

                        if count.count == 0:
                            logging.warning(f"No embeddings found for book {book_id}. The book may not be indexed.")
                            return Answer(
                                query_id=query_id,
                                response="This book hasn't been indexed yet. Please ensure the book content has been embedded in the vector store.",
                                citations=[],
                                confidence_score=0.0,
                                answer_id=UUID(int=0)  # Placeholder ID
                            )
                        else:
                            logging.warning(f"Qdrant search failed but embeddings exist for book {book_id}")
                    except Exception as count_error:
                        logging.error(f"Error checking book embeddings count: {count_error}")

                    return Answer(
                        query_id=query_id,
                        response="I'm currently unable to search the book content. Please try again later.",
                        citations=[],
                        confidence_score=0.0,
                        answer_id=UUID(int=0)  # Placeholder ID
                    )

                # Extract relevant chunks from search results
                relevant_chunks = []
                for result in search_results:
                    try:
                        chunk_data = result.payload
                        chunk = Chunk(
                            chunk_id=UUID(chunk_data['chunk_id']),
                            book_id=UUID(chunk_data['book_id']),
                            content=chunk_data['content'],
                            chunk_index=chunk_data['chunk_index'],
                            metadata=chunk_data.get('metadata', {}),
                            vector_id=result.id
                        )
                        relevant_chunks.append(chunk)
                    except Exception as e:
                        logging.warning(f"Error processing search result: {e}. Skipping this result.")

                # Generate the answer using the relevant chunks
                response_text, citations = await self._generate_answer_with_context(
                    query.question,
                    relevant_chunks,
                    query.rag_mode
                )

            # Create an Answer object
            answer_data = AnswerCreate(
                query_id=query_id,
                response=response_text,
                citations=citations,
                confidence_score=0.9  # Placeholder confidence score
            )

            # For now, we'll return the answer object without storing it in DB
            # In a full implementation, we would store it using a service
            return Answer(
                query_id=answer_data.query_id,
                response=answer_data.response,
                citations=answer_data.citations,
                confidence_score=answer_data.confidence_score,
                answer_id=UUID(int=0)  # Placeholder ID
            )
        except Exception as e:
            logging.error(f"Unexpected error processing query: {e}")
            # Return a user-friendly error response instead of raising an exception
            return Answer(
                query_id=uuid4(),  # Generate a new query_id even for error responses
                response="I encountered an error while processing your query. Please try again later.",
                citations=[],
                confidence_score=0.0,
                answer_id=UUID(int=0)  # Placeholder ID
            )

    async def _create_selection_chunks(self, selection_context: str) -> List[Chunk]:
        """Create chunks from the provided selection context"""
        # Create a pseudo-chunk from the selection context
        from uuid import UUID

        return [
            Chunk(
                chunk_id=UUID(int=0),  # Placeholder ID
                book_id=UUID(int=0),   # Placeholder ID
                content=selection_context,
                chunk_index=0,
                metadata={},
                vector_id="placeholder"  # Placeholder vector ID
            )
        ]

    async def _generate_answer_with_context(
        self,
        question: str,
        relevant_chunks: List[Chunk],
        rag_mode: RAGMode
    ) -> tuple[str, List[Citation]]:
        """Generate an answer based on the question and relevant chunks with proper citation tracking"""
        # Combine relevant chunks to form the context
        context_parts = []
        citations = []

        for idx, chunk in enumerate(relevant_chunks):
            # Add the chunk to context for the LLM
            context_parts.append(f"Context {idx+1}: {chunk.content}")

            # Create a citation for each chunk used, tracking it properly
            citations.append(Citation(
                chunk_id=chunk.chunk_id,
                text=chunk.content[:100] + "..." if len(chunk.content) > 100 else chunk.content,
                page_number=chunk.metadata.get('page_number') if chunk.metadata else None
            ))

        context = "\n\n".join(context_parts)

        # Create the prompt for the LLM
        if rag_mode == RAGMode.SELECTION_ONLY:
            prompt = f"""
            Answer the question based only on the following selected context.
            Do not use any other information.

            Selected Context:
            {context}

            Question: {question}

            Answer:
            """
        else:
            prompt = f"""
            Answer the question based on the following context.

            Context:
            {context}

            Question: {question}

            Answer:
            """

        try:
            if settings.cohere_api_key:
                # Generate content using Cohere Chat API (replaces the deprecated Generate API)
                response = await self.co.chat(
                    message=prompt,  # Changed from 'prompt' to 'message' for Chat API
                    max_tokens=500,
                    temperature=0.3,
                    # stop_sequences parameter is not available in chat API, using other parameters instead
                )

                if response and response.text:
                    answer_text = response.text.strip()
                else:
                    # Fallback response if Cohere doesn't return content
                    answer_text = "This is a placeholder response. The actual implementation would generate an answer using the provided context and question."
            else:
                # Fallback response if no API key is provided
                answer_text = "This is a placeholder response. The actual implementation would generate an answer using the provided context and question."
        except Exception as e:
            logging.error(f"Error generating answer with Cohere: {e}")
            # Fallback response
            answer_text = "I encountered an error while generating the answer. Please try again later."

        return answer_text, citations

    async def ensure_book_indexed(self, book_id: UUID, book_content: str, chunk_size: int = 1000) -> bool:
        """Ensure a book is properly indexed in the vector store"""
        try:
            # Import here to avoid circular dependencies
            from ..core.book_indexer import book_indexer

            # Use the BookIndexer to handle the indexing
            return await book_indexer.index_book(book_id, book_content, chunk_size)
        except Exception as e:
            logging.error(f"Error indexing book: {e}")
            return False

    def _chunk_text(self, text: str, chunk_size: int) -> List[str]:
        """Break text into chunks of specified size"""
        tokens = self.enc.encode(text)
        chunks = []

        for i in range(0, len(tokens), chunk_size):
            chunk_tokens = tokens[i:i + chunk_size]
            chunk_text = self.enc.decode(chunk_tokens)
            chunks.append(chunk_text)

        return chunks