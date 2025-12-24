from typing import Optional
from uuid import UUID
import logging
from datetime import datetime, timedelta
from ..models.session import SessionCreate
from ..services.session_service import SessionService
from ..services.rag_service import RAGService
from ..models.query import QueryCreate, RAGMode


class ChatService:
    def __init__(self, session_service=None):
        self.session_service = session_service or SessionService()
        self.rag_service = RAGService(session_service=self.session_service)

    async def ensure_valid_session(self, session_id: Optional[UUID], book_id: Optional[UUID] = None) -> UUID:
        """Ensure we have a valid session ID, creating one if needed"""
        logging.info(f"Ensuring valid session, received session_id: {session_id}, book_id: {book_id}")

        if session_id is None:
            logging.info("No session_id provided, creating new session")
            # Create a new session
            if book_id is None:
                # If no book_id provided, we'll create a generic session
                session_data = SessionCreate(book_id=None, user_id=None)
            else:
                session_data = SessionCreate(book_id=book_id, user_id=None)

            new_session = await self.session_service.create_session(session_data)
            logging.info(f"Created new session with ID: {new_session.session_id}")
            return new_session.session_id

        # Validate existing session
        session = await self.session_service.get_session(session_id)
        if session is None:
            # Session has expired or doesn't exist, create a new one
            logging.info(f"Session {session_id} not found or expired, creating new session")
            if book_id is None:
                # Use the original book_id from the expired session if available
                session_data = SessionCreate(book_id=None, user_id=None)
            else:
                session_data = SessionCreate(book_id=book_id, user_id=None)
            new_session = await self.session_service.create_session(session_data)
            logging.info(f"Created replacement session with ID: {new_session.session_id}")
            return new_session.session_id

        logging.info(f"Valid session found: {session_id}")
        return session_id

    async def process_query_with_session_handling(self, session_id: Optional[UUID], question: str,
                                                 rag_mode: RAGMode = RAGMode.GLOBAL,
                                                 selection_context: Optional[str] = None,
                                                 book_id: Optional[UUID] = None) -> tuple:
        """Process a query with automatic session management"""
        logging.info(f"Processing query with session management, session_id: {session_id}, book_id: {book_id}")

        # Ensure we have a valid session
        valid_session_id = await self.ensure_valid_session(session_id, book_id)

        # Create query data
        query_data = QueryCreate(
            session_id=valid_session_id,
            question=question,
            rag_mode=rag_mode,
            selection_context=selection_context
        )

        # Process the query with the RAG service
        answer = await self.rag_service.process_query(query_data)

        return answer, valid_session_id