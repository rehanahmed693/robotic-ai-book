from pydantic import BaseModel, model_validator, field_validator
from typing import List, Optional, Union, Any, Dict
from uuid import UUID
from datetime import datetime
from ..models.query import RAGMode



class StartSessionRequest(BaseModel):
    book_id: UUID
    user_id: Optional[UUID] = None


class StartSessionResponse(BaseModel):
    session_id: UUID
    message: str


class QueryRequest(BaseModel):
    session_id: Optional[UUID] = None  # Make session_id optional to allow automatic session creation
    question: Optional[str] = None
    message: Optional[str] = None
    rag_mode: RAGMode = RAGMode.GLOBAL
    selection_context: Optional[str] = None
    book_id: Optional[UUID] = None  # Add book_id for automatic session creation

    @field_validator('session_id', mode='before')
    @classmethod
    def validate_session_id(cls, v):
        """Validate session_id is a proper UUID string if provided"""
        if v is None:
            return v  # Allow None for optional field
        if isinstance(v, str):
            try:
                # Try to parse the string as a UUID
                UUID(v)
                return v
            except ValueError:
                raise ValueError(f"Invalid UUID format for session_id: {v}. Expected format: xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx")
        return v

    @model_validator(mode='before')
    @classmethod
    def validate_fields(cls, values: Dict[str, Any]) -> Dict[str, Any]:
        # Support both 'message' and 'question' field names for compatibility
        if isinstance(values, dict):
            # Check if we have both 'message' and 'question' - prefer 'question'
            if 'message' in values and 'question' not in values:
                values['question'] = values.pop('message')
            elif 'question' in values and 'message' in values:
                # If both are provided, prefer 'question' and remove 'message'
                values.pop('message', None)

            # At least one of 'question' or 'message' must be provided
            if not values.get('question') and not values.get('message'):
                raise ValueError("Either 'question' or 'message' field is required in the request body")

            # session_id is now optional as it will be created automatically if needed

        return values


class Citation(BaseModel):
    chunk_id: UUID
    text: str
    page_number: Optional[int] = None


class QueryResponse(BaseModel):
    answer: str
    citations: List[Citation]
    confidence_score: Optional[float] = None
    session_id: Optional[UUID] = None


class GetHistoryRequest(BaseModel):
    session_id: UUID


class ConversationEntry(BaseModel):
    role: str  # "user" or "assistant"
    content: str
    timestamp: datetime
    citations: Optional[List[Citation]] = None


class GetHistoryResponse(BaseModel):
    history: List[ConversationEntry]


class EndSessionRequest(BaseModel):
    session_id: UUID


class EndSessionResponse(BaseModel):
    message: str


class ListBooksResponse(BaseModel):
    books: List[dict]  # Using dict to avoid circular import issues


class GetBookResponse(BaseModel):
    book_id: UUID
    title: str
    author: str
    description: Optional[str] = None
    chunk_count: Optional[int] = None
    created_at: datetime
    updated_at: datetime