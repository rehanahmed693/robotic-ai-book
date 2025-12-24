from typing import List, Dict, Any
from uuid import UUID
import logging
from ..vector_store.qdrant_client import qdrant_client


async def store_chunks_in_qdrant(book_id: str, chunks_data: List[Dict[str, Any]]) -> bool:
    """Store chunks in Qdrant vector database"""
    try:
        await qdrant_client.upsert_vectors(book_id, chunks_data)
        return True
    except Exception as e:
        logging.error(f"Error storing chunks in Qdrant: {e}")
        return False


async def search_in_qdrant(query_vector: List[float], book_id: str = None, limit: int = 10) -> List[Dict[str, Any]]:
    """Search for similar chunks in Qdrant"""
    try:
        results = await qdrant_client.search_vectors(query_vector, limit, book_id)
        
        # Format results to a standard structure
        formatted_results = []
        for result in results:
            formatted_results.append({
                'chunk_id': result.payload.get('chunk_id'),
                'book_id': result.payload.get('book_id'),
                'content': result.payload.get('content'),
                'chunk_index': result.payload.get('chunk_index'),
                'metadata': result.payload.get('metadata', {}),
                'score': result.score
            })
        
        return formatted_results
    except Exception as e:
        logging.error(f"Error searching in Qdrant: {e}")
        raise


async def delete_book_from_qdrant(book_id: str) -> bool:
    """Delete all vectors associated with a book from Qdrant"""
    try:
        await qdrant_client.delete_by_book_id(book_id)
        return True
    except Exception as e:
        logging.error(f"Error deleting book from Qdrant: {e}")
        return False