from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Optional, Dict, Any
from uuid import UUID
import logging
from ..core.config import settings


class QdrantClientWrapper:
    def __init__(self):
        # Initialize the client only when first used
        self._client = None
        self.collection_name = "book_chunks"

    @property
    def client(self):
        if self._client is None:
            if not settings.qdrant_url:
                raise ValueError("Qdrant URL is not configured. Please set QDRANT_URL in your environment variables.")
            if "https" not in settings.qdrant_url and "http" not in settings.qdrant_url:
                raise ValueError(f"Invalid Qdrant URL: {settings.qdrant_url}. URL must include http or https scheme.")

            self._client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                prefer_grpc=True
            )
        return self._client

    async def create_collection(self, vector_size: int = 4096):  # Default to Cohere embedding size
        """Create the collection for storing book chunks if it doesn't exist"""
        try:
            # Check if collection exists
            collection_exists = self.client.collection_exists(collection_name=self.collection_name)

            # Create collection if it doesn't exist
            if not collection_exists:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=vector_size,  # Size depends on embedding model (4096 for Cohere, 1536 for OpenAI)
                        distance=models.Distance.COSINE
                    )
                )
                logging.info(f"Created Qdrant collection: {self.collection_name} with vector size: {vector_size}")
            else:
                logging.info(f"Qdrant collection {self.collection_name} already exists")

            # Create an index on the book_id field to improve search performance
            try:
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="book_id",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )
                logging.info("Created index on 'book_id' field in Qdrant collection")
            except Exception as index_error:
                # This might fail if the index already exists, which is fine
                logging.info(f"Index on 'book_id' field may already exist: {index_error}")
        except Exception as e:
            logging.error(f"Error creating collection: {e}")
            raise

    async def upsert_vectors(self, book_id: str, vectors_data: List[Dict[str, Any]]):
        """Upsert vectors into the collection"""
        try:
            points = []
            for idx, data in enumerate(vectors_data):
                point = models.PointStruct(
                    id=str(data['vector_id']),
                    vector=data['embedding'],
                    payload={
                        "book_id": book_id,
                        "chunk_id": data['chunk_id'],
                        "content": data['content'],
                        "chunk_index": data['chunk_index'],
                        "metadata": data.get('metadata', {})
                    }
                )
                points.append(point)

            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            logging.info(f"Upserted {len(points)} vectors for book {book_id}")
        except Exception as e:
            logging.error(f"Error upserting vectors: {e}")
            raise

    async def search_vectors(self, query_vector: List[float], limit: int = 10, book_id: Optional[str] = None):
        """Search for similar vectors in the collection"""
        try:
            # Prepare filters
            filter_conditions = []
            if book_id:
                filter_conditions.append(
                    models.FieldCondition(
                        key="book_id",
                        match=models.MatchValue(value=book_id)
                    )
                )

            search_filter = models.Filter(
                must=filter_conditions
            ) if filter_conditions else None

            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                query_filter=search_filter,
                limit=limit,
                with_payload=True
            )

            return results
        except Exception as e:
            logging.error(f"Error searching vectors: {e}")
            # Return an empty list instead of raising an exception to prevent 500 errors
            return []

    async def delete_by_book_id(self, book_id: str):
        """Delete all vectors associated with a specific book ID"""
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.FilterSelector(
                    filter=models.Filter(
                        must=[
                            models.FieldCondition(
                                key="book_id",
                                match=models.MatchValue(value=book_id)
                            )
                        ]
                    )
                )
            )
            logging.info(f"Deleted vectors for book {book_id}")
        except Exception as e:
            logging.error(f"Error deleting vectors for book {book_id}: {e}")
            raise


# Global Qdrant client instance (lazy initialization)
qdrant_client = QdrantClientWrapper()