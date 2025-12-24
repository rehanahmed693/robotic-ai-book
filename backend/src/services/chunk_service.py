from typing import List, Optional
from uuid import UUID
import logging
from ..models.chunk import Chunk, ChunkCreate, ChunkUpdate
from ..database.connections import db_connection


class ChunkService:
    def __init__(self):
        pass

    async def create_chunk(self, chunk_data: ChunkCreate) -> Chunk:
        """Create a new chunk in the database"""
        conn = await db_connection.get_connection()
        try:
            # Generate a new UUID for the chunk
            from uuid import uuid4
            chunk_id = uuid4()

            # Insert the chunk into the database
            query = """
                INSERT INTO chunks (chunk_id, book_id, content, chunk_index, metadata, created_at, updated_at)
                VALUES (%s, %s, %s, %s, %s, NOW(), NOW())
                RETURNING chunk_id, book_id, content, chunk_index, metadata, created_at, updated_at
            """
            cur = await conn.execute(
                query,
                (chunk_id, chunk_data.book_id, chunk_data.content, chunk_data.chunk_index, chunk_data.metadata)
            )
            record = await cur.fetchone()

            # Return the created chunk
            return Chunk(
                chunk_id=record[0],
                book_id=record[1],
                content=record[2],
                chunk_index=record[3],
                metadata=record[4],
                created_at=record[5],
                updated_at=record[6]
            )
        except Exception as e:
            logging.error(f"Error creating chunk: {e}")
            raise
        finally:
            await db_connection.release_connection(conn)

    async def get_chunk(self, chunk_id: UUID) -> Optional[Chunk]:
        """Retrieve a chunk by its ID"""
        conn = await db_connection.get_connection()
        try:
            query = """
                SELECT chunk_id, book_id, content, chunk_index, metadata, created_at, updated_at
                FROM chunks
                WHERE chunk_id = %s
            """
            cur = await conn.execute(query, (chunk_id,))
            record = await cur.fetchone()

            if not record:
                return None

            return Chunk(
                chunk_id=record[0],
                book_id=record[1],
                content=record[2],
                chunk_index=record[3],
                metadata=record[4],
                created_at=record[5],
                updated_at=record[6]
            )
        except Exception as e:
            logging.error(f"Error retrieving chunk: {e}")
            raise
        finally:
            await db_connection.release_connection(conn)

    async def get_chunks_by_book(self, book_id: UUID) -> List[Chunk]:
        """Retrieve all chunks associated with a specific book"""
        conn = await db_connection.get_connection()
        try:
            query = """
                SELECT chunk_id, book_id, content, chunk_index, metadata, created_at, updated_at
                FROM chunks
                WHERE book_id = %s
                ORDER BY chunk_index
            """
            cur = await conn.execute(query, (book_id,))
            records = await cur.fetchall()

            chunks = []
            for record in records:
                chunks.append(Chunk(
                    chunk_id=record[0],
                    book_id=record[1],
                    content=record[2],
                    chunk_index=record[3],
                    metadata=record[4],
                    created_at=record[5],
                    updated_at=record[6]
                ))

            return chunks
        except Exception as e:
            logging.error(f"Error retrieving chunks for book: {e}")
            raise
        finally:
            await db_connection.release_connection(conn)

    async def update_chunk(self, chunk_id: UUID, chunk_data: ChunkUpdate) -> Optional[Chunk]:
        """Update a chunk's information"""
        conn = await db_connection.get_connection()
        try:
            # Build the dynamic update query
            update_fields = []
            values = []
            value_index = 2  # Start from $2 since $1 is chunk_id

            if chunk_data.content is not None:
                update_fields.append(f"content = %s")
                values.append(chunk_data.content)
                value_index += 1

            if chunk_data.chunk_index is not None:
                update_fields.append(f"chunk_index = %s")
                values.append(chunk_data.chunk_index)
                value_index += 1

            if chunk_data.metadata is not None:
                update_fields.append(f"metadata = %s")
                values.append(chunk_data.metadata)
                value_index += 1

            if not update_fields:
                # Nothing to update, just update the timestamp
                query = """
                    UPDATE chunks
                    SET updated_at = NOW()
                    WHERE chunk_id = %s
                    RETURNING chunk_id, book_id, content, chunk_index, metadata, created_at, updated_at
                """
                cur = await conn.execute(query, (chunk_id,))
                record = await cur.fetchone()
            else:
                # Add chunk_id and update timestamp
                values = [chunk_id] + values + [chunk_id]

                query = f"""
                    UPDATE chunks
                    SET {', '.join(update_fields)}, updated_at = NOW()
                    WHERE chunk_id = %s
                    RETURNING chunk_id, book_id, content, chunk_index, metadata, created_at, updated_at
                """

                cur = await conn.execute(query, tuple(values))
                record = await cur.fetchone()

            if not record:
                return None

            return Chunk(
                chunk_id=record[0],
                book_id=record[1],
                content=record[2],
                chunk_index=record[3],
                metadata=record[4],
                created_at=record[5],
                updated_at=record[6]
            )
        except Exception as e:
            logging.error(f"Error updating chunk: {e}")
            raise
        finally:
            await db_connection.release_connection(conn)

    async def delete_chunk(self, chunk_id: UUID) -> bool:
        """Delete a chunk from the database"""
        conn = await db_connection.get_connection()
        try:
            query = "DELETE FROM chunks WHERE chunk_id = %s"
            result = await conn.execute(query, (chunk_id,))

            # Check if a row was actually deleted
            result_parts = result.split()
            rows_affected = int(result_parts[-1]) if len(result_parts) > 0 and result_parts[-1].isdigit() else 0

            return rows_affected > 0
        except Exception as e:
            logging.error(f"Error deleting chunk: {e}")
            raise
        finally:
            await db_connection.release_connection(conn)

    async def delete_chunks_by_book(self, book_id: UUID) -> int:
        """Delete all chunks associated with a specific book"""
        conn = await db_connection.get_connection()
        try:
            query = "DELETE FROM chunks WHERE book_id = %s"
            result = await conn.execute(query, (book_id,))

            # Extract the number of affected rows
            result_parts = result.split()
            rows_affected = int(result_parts[-1]) if len(result_parts) > 0 and result_parts[-1].isdigit() else 0

            return rows_affected
        except Exception as e:
            logging.error(f"Error deleting chunks for book: {e}")
            raise
        finally:
            await db_connection.release_connection(conn)