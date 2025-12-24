from typing import List, Optional
from uuid import UUID
import logging
from ..models.book import Book, BookCreate, BookUpdate
from ..database.connections import db_connection


class BookService:
    def __init__(self):
        pass

    async def create_book(self, book_data: BookCreate) -> Book:
        """Create a new book in the database"""
        conn = await db_connection.get_connection()
        try:
            # Generate a new UUID for the book
            from uuid import uuid4
            book_id = uuid4()

            # Insert the book into the database
            query = """
                INSERT INTO books (book_id, title, author, content, description, created_at, updated_at)
                VALUES (%s, %s, %s, %s, %s, NOW(), NOW())
                RETURNING book_id, title, author, description, created_at, updated_at
            """
            cur = await conn.execute(
                query,
                (book_id, book_data.title, book_data.author, book_data.content, book_data.description)
            )
            record = await cur.fetchone()

            # Return the created book
            return Book(
                book_id=record[0],
                title=record[1],
                author=record[2],
                description=record[3],
                created_at=record[4],
                updated_at=record[5]
            )
        except Exception as e:
            logging.error(f"Error creating book: {e}")
            raise
        finally:
            await db_connection.release_connection(conn)

    async def get_book(self, book_id: UUID) -> Optional[Book]:
        """Retrieve a book by its ID"""
        conn = await db_connection.get_connection()
        try:
            query = """
                SELECT book_id, title, author, description, created_at, updated_at
                FROM books
                WHERE book_id = %s
            """
            cur = await conn.execute(query, (book_id,))
            record = await cur.fetchone()

            if not record:
                return None

            # Get chunk count for the book
            count_query = """
                SELECT COUNT(*) as chunk_count
                FROM chunks
                WHERE book_id = %s
            """
            count_cur = await conn.execute(count_query, (book_id,))
            count_record = await count_cur.fetchone()

            return Book(
                book_id=record[0],
                title=record[1],
                author=record[2],
                description=record[3],
                created_at=record[4],
                updated_at=record[5],
                chunk_count=count_record[0]
            )
        except Exception as e:
            logging.error(f"Error retrieving book: {e}")
            raise
        finally:
            await db_connection.release_connection(conn)

    async def list_books(self) -> List[Book]:
        """List all books in the database"""
        conn = await db_connection.get_connection()
        try:
            query = """
                SELECT book_id, title, author, description, created_at, updated_at
                FROM books
            """
            cur = await conn.execute(query)
            records = await cur.fetchall()

            books = []
            for record in records:
                # Get chunk count for each book
                count_query = """
                    SELECT COUNT(*) as chunk_count
                    FROM chunks
                    WHERE book_id = %s
                """
                count_cur = await conn.execute(count_query, (record[0],))
                count_record = await count_cur.fetchone()

                books.append(Book(
                    book_id=record[0],
                    title=record[1],
                    author=record[2],
                    description=record[3],
                    created_at=record[4],
                    updated_at=record[5],
                    chunk_count=count_record[0]
                ))

            return books
        except Exception as e:
            logging.error(f"Error listing books: {e}")
            raise
        finally:
            await db_connection.release_connection(conn)

    async def update_book(self, book_id: UUID, book_data: BookUpdate) -> Optional[Book]:
        """Update a book's information"""
        conn = await db_connection.get_connection()
        try:
            # Build the dynamic update query
            update_fields = []
            values = []
            value_index = 2  # Start from $2 since $1 is book_id

            if book_data.title is not None:
                update_fields.append(f"title = %s")
                values.append(book_data.title)
                value_index += 1

            if book_data.author is not None:
                update_fields.append(f"author = %s")
                values.append(book_data.author)
                value_index += 1

            if book_data.description is not None:
                update_fields.append(f"description = %s")
                values.append(book_data.description)
                value_index += 1

            if not update_fields:
                # Nothing to update
                return await self.get_book(book_id)

            # Add book_id and updated_at to values
            values = [book_id] + values + [book_id]  # book_id for WHERE clause

            query = f"""
                UPDATE books
                SET {', '.join(update_fields)}, updated_at = NOW()
                WHERE book_id = %s
                RETURNING book_id, title, author, description, created_at, updated_at
            """

            cur = await conn.execute(query, tuple(values))
            record = await cur.fetchone()

            if not record:
                return None

            # Get chunk count for the book
            count_query = """
                SELECT COUNT(*) as chunk_count
                FROM chunks
                WHERE book_id = %s
            """
            count_cur = await conn.execute(count_query, (book_id,))
            count_record = await count_cur.fetchone()

            return Book(
                book_id=record[0],
                title=record[1],
                author=record[2],
                description=record[3],
                created_at=record[4],
                updated_at=record[5],
                chunk_count=count_record[0]
            )
        except Exception as e:
            logging.error(f"Error updating book: {e}")
            raise
        finally:
            await db_connection.release_connection(conn)

    async def delete_book(self, book_id: UUID) -> bool:
        """Delete a book and its associated chunks"""
        conn = await db_connection.get_connection()
        try:
            # Delete associated chunks first (due to foreign key constraints)
            delete_chunks_query = "DELETE FROM chunks WHERE book_id = %s"
            await conn.execute(delete_chunks_query, (book_id,))

            # Then delete the book
            delete_book_query = "DELETE FROM books WHERE book_id = %s"
            result = await conn.execute(delete_book_query, (book_id,))

            # Check if a row was actually deleted
            result_parts = result.split()
            rows_affected = int(result_parts[-1]) if len(result_parts) > 0 and result_parts[-1].isdigit() else 0

            return rows_affected > 0
        except Exception as e:
            logging.error(f"Error deleting book: {e}")
            raise
        finally:
            await db_connection.release_connection(conn)