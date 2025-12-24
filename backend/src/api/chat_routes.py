from fastapi import APIRouter, HTTPException
import logging
from uuid import UUID

from .schemas import (
    StartSessionRequest,
    StartSessionResponse,
    QueryRequest,
    QueryResponse,
    GetHistoryRequest,
    GetHistoryResponse,
    EndSessionRequest,
    EndSessionResponse
)
from ..services.chat_service import ChatService
from ..services.session_service import SessionService
from ..services.rag_service import RAGService
from ..models.query import QueryCreate, RAGMode
from ..models.session import SessionCreate

router = APIRouter()

# Initialize services
session_service = SessionService()  # Create session service first
chat_service = ChatService(session_service=session_service)  # Pass it to chat service
rag_service = RAGService(session_service=session_service)  # Pass it to rag service too


@router.post("/chat/start", response_model=StartSessionResponse)
async def start_chat_session(request: StartSessionRequest):
    """Create a new chat session for interacting with a book"""
    try:
        # Create session data
        session_data = SessionCreate(
            book_id=request.book_id,
            user_id=request.user_id
        )

        # Create the session using the service
        session = await session_service.create_session(session_data)

        return StartSessionResponse(
            session_id=session.session_id,
            message="Session created successfully"
        )
    except Exception as e:
        logging.error(f"Error creating session: {e}")
        # Instead of raising a 500 error, we'll return a more helpful message
        # This could be due to database issues, invalid UUIDs, etc.
        raise HTTPException(status_code=500, detail=f"Failed to create session: {str(e)}")


@router.post("/chat/query", response_model=QueryResponse)
async def submit_query(request: QueryRequest):
    """Submit a question to the RAG system and get an answer with citations"""
    try:
        # Validate the session exists and is active
        session = await session_service.get_session(request.session_id)
        if not session:
            # If session doesn't exist, try to create one with the provided book_id
            logging.info(f"Session {request.session_id} not found, creating new session with book_id: {request.book_id}")

            from ..models.session import SessionCreate
            session_data = SessionCreate(
                book_id=request.book_id,  # Use the book_id from the request if available
                user_id=None
            )
            new_session = await session_service.create_session(session_data)
            actual_session_id = new_session.session_id
        else:
            actual_session_id = request.session_id

        # The question field should already be populated by the validator
        question_text = request.question
        if not question_text:
            raise HTTPException(status_code=422, detail="Either 'question' or 'message' field is required")

        # Create query data with the actual session ID
        query_data = QueryCreate(
            session_id=actual_session_id,
            question=question_text,
            rag_mode=request.rag_mode,
            selection_context=request.selection_context
        )

        # Process the query with the RAG service
        answer = await rag_service.process_query(query_data)

        return QueryResponse(
            answer=answer.response,
            citations=answer.citations,
            confidence_score=answer.confidence_score
        )
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logging.error(f"Error processing query: {e}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.post("/chat/message", response_model=QueryResponse)
async def send_message(request: QueryRequest):
    """Send a message and get a response with automatic session management.

    This endpoint handles session creation and expiration automatically,
    making it easier for frontend implementations that don't need to manage
    sessions explicitly.
    """
    try:
        # The question field should already be populated by the validator
        question_text = request.question
        if not question_text:
            raise HTTPException(status_code=422, detail="Either 'question' or 'message' field is required")

        logging.info(f"Processing message with session_id: {request.session_id}, book_id: {request.book_id}")

        # Process query with automatic session management
        # If session_id is provided but invalid, the service will create a new one
        answer, actual_session_id = await chat_service.process_query_with_session_handling(
            session_id=request.session_id,
            question=question_text,
            rag_mode=request.rag_mode,
            selection_context=request.selection_context,
            book_id=request.book_id
        )

        logging.info(f"Message processed successfully with actual session_id: {actual_session_id}")
        return QueryResponse(
            answer=answer.response,
            citations=answer.citations,
            confidence_score=answer.confidence_score,
            session_id=actual_session_id  # Include session ID in response
        )
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logging.error(f"Error processing message: {e}")
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.get("/chat/history", response_model=GetHistoryResponse)
async def get_conversation_history(
    session_id: UUID = None,
    limit: int = 10,
    offset: int = 0
):
    """Retrieve the conversation history for a session"""
    try:
        # Validate that session_id is provided
        if not session_id:
            raise HTTPException(status_code=422, detail="session_id is required")

        # For now, return an empty history since the full implementation would require
        # a ConversationService and database storage for conversation history
        # This is a placeholder implementation that just returns an empty list
        # In a full implementation, this would fetch from a database
        return GetHistoryResponse(history=[])
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logging.error(f"Error retrieving conversation history: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.post("/chat/end", response_model=EndSessionResponse)
async def end_chat_session(request: EndSessionRequest):
    """End a chat session"""
    try:
        # End the session
        success = await session_service.end_session(request.session_id)

        if not success:
            raise HTTPException(status_code=404, detail="Session not found")

        return EndSessionResponse(message="Session ended successfully")
    except Exception as e:
        logging.error(f"Error ending session: {e}")
        raise HTTPException(status_code=500, detail="Internal server error")