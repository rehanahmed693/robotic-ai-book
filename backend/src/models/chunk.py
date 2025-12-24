from pydantic import BaseModel, Field
from typing import Optional, Dict, Any
from uuid import UUID, uuid4
from datetime import datetime
from .base import BaseSchema


class ChunkBase(BaseModel):
    content: str
    chunk_index: int
    metadata: Optional[Dict[str, Any]] = Field(default_factory=dict)


class ChunkCreate(ChunkBase):
    book_id: UUID


class ChunkUpdate(BaseModel):
    content: Optional[str] = None
    chunk_index: Optional[int] = None
    metadata: Optional[Dict[str, Any]] = None


class ChunkInDBBase(ChunkBase):
    chunk_id: UUID = Field(default_factory=uuid4)
    book_id: UUID
    vector_id: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)


class Chunk(ChunkInDBBase):
    chunk_id: UUID
    book_id: UUID