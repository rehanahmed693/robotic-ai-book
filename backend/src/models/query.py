from pydantic import BaseModel, Field
from typing import Optional, List
from uuid import UUID, uuid4
from datetime import datetime
from enum import Enum


class RAGMode(str, Enum):
    GLOBAL = "global"
    SELECTION_ONLY = "selection_only"


class QueryBase(BaseModel):
    session_id: UUID
    question: str
    rag_mode: RAGMode = RAGMode.GLOBAL
    selection_context: Optional[str] = None


class QueryCreate(QueryBase):
    pass


class QueryUpdate(BaseModel):
    rag_mode: Optional[RAGMode] = None
    selection_context: Optional[str] = None


class QueryInDBBase(QueryBase):
    query_id: UUID = Field(default_factory=uuid4)
    timestamp: datetime = Field(default_factory=datetime.now)


class Query(QueryInDBBase):
    query_id: UUID
    session_id: UUID