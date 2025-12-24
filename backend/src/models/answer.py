from pydantic import BaseModel, Field
from typing import Optional, List
from uuid import UUID, uuid4
from datetime import datetime


class Citation(BaseModel):
    chunk_id: UUID
    text: str
    page_number: Optional[int] = None


class AnswerBase(BaseModel):
    query_id: UUID
    response: str
    citations: List[Citation] = Field(default_factory=list)
    confidence_score: Optional[float] = None


class AnswerCreate(AnswerBase):
    pass


class AnswerUpdate(BaseModel):
    response: Optional[str] = None
    citations: Optional[List[Citation]] = None
    confidence_score: Optional[float] = None


class AnswerInDBBase(AnswerBase):
    answer_id: UUID = Field(default_factory=uuid4)
    timestamp: datetime = Field(default_factory=datetime.now)


class Answer(AnswerInDBBase):
    answer_id: UUID
    query_id: UUID