from typing import List
import logging
from uuid import UUID
from ..models.chunk import ChunkCreate
from ..services.chunk_service import ChunkService
from ..core.embeddings import get_embeddings
from ..vector_store.chunk_storage import store_chunks_in_qdrant
import tiktoken


class BookIndexer:
    def __init__(self):
        self.chunk_service = ChunkService()
        self.enc = tiktoken.encoding_for_model("gpt-3.5-turbo")
    
    async def index_book(self, book_id: UUID, book_content: str, chunk_size: int = 1000) -> bool:
        """Index a book by breaking it into chunks, generating embeddings, and storing in vector DB"""
        try:
            # Break the book content into chunks
            chunks = self._chunk_text(book_content, chunk_size)

            # Prepare data for vector storage
            vectors_data = []
            chunk_objects = []

            # Determine the embedding vector size (this will help ensure Qdrant collection is created with the right size)
            from ..core.embeddings import get_embedding_vector_size
            vector_size = get_embedding_vector_size(use_cohere=True)  # Use Cohere since that's what we have available

            # Ensure the Qdrant collection exists with the correct vector size
            from ..vector_store.qdrant_client import qdrant_client
            await qdrant_client.create_collection(vector_size=vector_size)

            for idx, chunk_text in enumerate(chunks):
                # Create chunk object for database storage
                chunk_obj = ChunkCreate(
                    content=chunk_text,
                    chunk_index=idx,
                    book_id=book_id,
                    metadata={"chunk_index": idx}  # Add any additional metadata
                )
                chunk_objects.append(chunk_obj)

                # Generate embeddings for the chunk
                embedding = await get_embeddings(chunk_text)

                import uuid
                # Prepare vector data for Qdrant with proper UUIDs
                vector_id = str(uuid.uuid4())
                chunk_id = str(uuid.uuid4())

                vector_data = {
                    'vector_id': vector_id,
                    'chunk_id': chunk_id,
                    'content': chunk_text,
                    'chunk_index': idx,
                    'embedding': embedding
                }
                vectors_data.append(vector_data)

            # Store chunks in the database (this would require creating the chunks first)
            # For now, we'll just use the vector storage
            for chunk_obj in chunk_objects:
                # await self.chunk_service.create_chunk(chunk_obj)  # Implement when chunk DB operations are ready
                pass

            # Store vectors in Qdrant
            success = await store_chunks_in_qdrant(str(book_id), vectors_data)

            if success:
                logging.info(f"Successfully indexed book {book_id} with {len(chunks)} chunks")
                return True
            else:
                logging.error(f"Failed to store vectors for book {book_id}")
                return False

        except Exception as e:
            logging.error(f"Error indexing book {book_id}: {e}")
            raise
    
    def _chunk_text(self, text: str, chunk_size: int) -> List[str]:
        """Break text into chunks of specified size"""
        tokens = self.enc.encode(text)
        chunks = []
        
        for i in range(0, len(tokens), chunk_size):
            chunk_tokens = tokens[i:i + chunk_size]
            chunk_text = self.enc.decode(chunk_tokens)
            chunks.append(chunk_text)
        
        return chunks


# Global instance of BookIndexer
book_indexer = BookIndexer()