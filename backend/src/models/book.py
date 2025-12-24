from pydantic import BaseModel, Field
from typing import Optional
from uuid import UUID, uuid4
from datetime import datetime
from .base import BaseSchema


class BookBase(BaseModel):
    title: str
    author: str
    description: Optional[str] = None


class BookCreate(BookBase):
    content: str


class BookUpdate(BaseModel):
    title: Optional[str] = None
    author: Optional[str] = None
    description: Optional[str] = None


class BookInDBBase(BookBase):
    book_id: UUID = Field(default_factory=uuid4)
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)


class Book(BookInDBBase):
    book_id: UUID
    content: Optional[str] = None  # Not always needed in responses
    chunk_count: Optional[int] = None


class BookWithContent(Book):
    content: str