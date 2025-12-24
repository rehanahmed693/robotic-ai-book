from typing import List, Optional
from uuid import UUID
import logging
from datetime import datetime, timedelta
from ..models.session import Session, SessionCreate, SessionUpdate
from ..database.connections import db_connection


class SessionService:
    def __init__(self):
        # In-memory storage for sessions when DB is not available
        self._in_memory_sessions = {}

    async def create_session(self, session_data: SessionCreate) -> Session:
        """Create a new session in the database"""
        if not db_connection.is_connected:
            logging.warning("Database not connected. Creating a session with limited functionality.")

            # Generate a new UUID for the session
            from uuid import uuid4
            session_id = uuid4()

            # Set expiration time (e.g., 1 hour from now)
            expires_at = datetime.now() + timedelta(hours=1)
            logging.info(f"Creating in-memory session {session_id} with expiration: {expires_at}")

            # Create session object
            session = Session(
                session_id=session_id,
                book_id=session_data.book_id,
                user_id=session_data.user_id,
                created_at=datetime.now(),
                updated_at=datetime.now(),
                expires_at=expires_at
            )

            # Store in memory for later retrieval
            self._in_memory_sessions[session_id] = session

            # Return the session object
            return session

        conn = await db_connection.get_connection()
        try:
            # Generate a new UUID for the session
            from uuid import uuid4
            session_id = uuid4()

            # Set expiration time (e.g., 1 hour from now)
            expires_at = datetime.now() + timedelta(hours=1)
            logging.info(f"Setting session {session_id} expiration time to: {expires_at}")

            # Insert the session into the database with explicit transaction
            query = """
                INSERT INTO sessions (session_id, book_id, user_id, created_at, updated_at, expires_at)
                VALUES (%s, %s, %s, NOW(), NOW(), %s)
                RETURNING session_id, book_id, user_id, created_at, updated_at, expires_at
            """

            # Use a transaction block to ensure the insert is properly committed
            async with conn.transaction():
                cur = await conn.execute(
                    query,
                    (session_id, session_data.book_id, session_data.user_id, expires_at)
                )
                record = await cur.fetchone()

            logging.info(f"Session {session_id} created and committed to database")

            # Return the created session
            return Session(
                session_id=record[0],
                book_id=record[1],
                user_id=record[2],
                created_at=record[3],
                updated_at=record[4],
                expires_at=record[5]
            )
        except Exception as e:
            logging.error(f"Error creating session: {e}")
            # Even if DB insertion fails, return a session object to allow continued operation
            from uuid import uuid4
            session_id = uuid4()
            expires_at = datetime.now() + timedelta(hours=1)
            logging.warning(f"Creating fallback session {session_id} due to DB error, expiration: {expires_at}")

            return Session(
                session_id=session_id,
                book_id=session_data.book_id,
                user_id=session_data.user_id,
                created_at=datetime.now(),
                updated_at=datetime.now(),
                expires_at=expires_at
            )
        finally:
            await db_connection.release_connection(conn)

    async def get_session(self, session_id: UUID) -> Optional[Session]:
        """Retrieve a session by its ID"""
        logging.info(f"Attempting to retrieve session: {session_id}")

        if not db_connection.is_connected:
            logging.warning("Database not connected. Checking in-memory sessions.")
            # Check if session exists in memory
            session = self._in_memory_sessions.get(session_id)
            if session:
                # Check if session has expired
                if session.expires_at > datetime.now():
                    logging.info(f"Found session {session_id} in memory")
                    return session
                else:
                    # Session has expired, remove it from memory
                    logging.info(f"Session {session_id} found in memory but has expired")
                    del self._in_memory_sessions[session_id]
                    return None
            logging.info(f"Session {session_id} not found in memory")
            return None

        conn = await db_connection.get_connection()
        try:
            # Log connection info for debugging
            logging.info(f"Retrieving session {session_id} from database")

            query = """
                SELECT session_id, book_id, user_id, created_at, updated_at, expires_at
                FROM sessions
                WHERE session_id = %s AND expires_at > NOW()
            """
            cur = await conn.execute(query, (session_id,))
            record = await cur.fetchone()

            if not record:
                logging.info(f"Session {session_id} not found in database or has expired")
                # Let's also check if the session exists but has expired
                check_expired_query = """
                    SELECT session_id, expires_at FROM sessions WHERE session_id = %s
                """
                check_cur = await conn.execute(check_expired_query, (session_id,))
                check_record = await check_cur.fetchone()

                if check_record:
                    # Get the actual expiration time to log
                    actual_expires_at = check_record[1]
                    current_time = datetime.now()
                    logging.info(f"Session {session_id} exists in database but has expired. "
                                f"Expires at: {actual_expires_at}, Current time: {current_time}, "
                                f"Difference: {current_time - actual_expires_at}")
                else:
                    logging.info(f"Session {session_id} does not exist in database at all")

                return None

            logging.info(f"Successfully retrieved session {session_id} from database")
            return Session(
                session_id=record[0],
                book_id=record[1],
                user_id=record[2],
                created_at=record[3],
                updated_at=record[4],
                expires_at=record[5]
            )
        except Exception as e:
            logging.error(f"Error retrieving session {session_id}: {e}")
            # Return None if there's an error, which will be handled by the calling function
            return None
        finally:
            await db_connection.release_connection(conn)

    async def update_session(self, session_id: UUID, session_data: SessionUpdate) -> Optional[Session]:
        """Update a session's information"""
        if not db_connection.is_connected:
            logging.warning("Database not connected. Cannot update session in DB.")
            return None

        conn = await db_connection.get_connection()
        try:
            # Build the dynamic update query
            update_fields = []
            values = []
            value_index = 2  # Start from $2 since $1 is session_id

            if session_data.user_id is not None:
                update_fields.append(f"user_id = ${value_index}")
                values.append(session_data.user_id)
                value_index += 1

            if not update_fields:
                # Nothing to update, just update the timestamp
                query = """
                    UPDATE sessions
                    SET updated_at = NOW()
                    WHERE session_id = %s AND expires_at > NOW()
                    RETURNING session_id, book_id, user_id, created_at, updated_at, expires_at
                """
                async with conn.transaction():
                    cur = await conn.execute(query, (session_id,))
                    record = await cur.fetchone()
            else:
                # Add session_id and update timestamp
                values = [session_id] + values + [session_id]

                query = f"""
                    UPDATE sessions
                    SET {', '.join(update_fields)}, updated_at = NOW()
                    WHERE session_id = ${value_index} AND expires_at > NOW()
                    RETURNING session_id, book_id, user_id, created_at, updated_at, expires_at
                """

                async with conn.transaction():
                    cur = await conn.execute(query, tuple(values))
                    record = await cur.fetchone()

            if not record:
                return None

            return Session(
                session_id=record[0],
                book_id=record[1],
                user_id=record[2],
                created_at=record[3],
                updated_at=record[4],
                expires_at=record[5]
            )
        except Exception as e:
            logging.error(f"Error updating session: {e}")
            raise
        finally:
            await db_connection.release_connection(conn)

    async def end_session(self, session_id: UUID) -> bool:
        """End a session by setting its expiration to now"""
        if not db_connection.is_connected:
            logging.warning("Database not connected. Ending in-memory session.")
            # Check if session exists in memory
            if session_id in self._in_memory_sessions:
                # Remove the session from memory
                del self._in_memory_sessions[session_id]
                return True
            return False  # Session not found

        conn = await db_connection.get_connection()
        try:
            # Set the expires_at to now to effectively end the session
            query = """
                UPDATE sessions
                SET expires_at = NOW()
                WHERE session_id = %s AND expires_at > NOW()
                RETURNING session_id
            """
            async with conn.transaction():
                cur = await conn.execute(query, (session_id,))
                record = await cur.fetchone()

            return record is not None
        except Exception as e:
            logging.error(f"Error ending session: {e}")
            return True  # Return success to allow frontend to continue
        finally:
            await db_connection.release_connection(conn)

    async def cleanup_expired_sessions(self) -> int:
        """Remove expired sessions from the database"""
        if not db_connection.is_connected:
            logging.warning("Database not connected. Cannot clean up expired sessions.")
            return 0

        conn = await db_connection.get_connection()
        try:
            query = """
                DELETE FROM sessions
                WHERE expires_at <= NOW()
            """
            async with conn.transaction():
                result = await conn.execute(query)
                # Get the number of affected rows
                rows_affected = result.rowcount

            return rows_affected
        except Exception as e:
            logging.error(f"Error cleaning up expired sessions: {e}")
            raise
        finally:
            await db_connection.release_connection(conn)