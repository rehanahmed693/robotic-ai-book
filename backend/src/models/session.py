from pydantic import BaseModel, Field
from typing import Optional
from uuid import UUID, uuid4
from datetime import datetime


class SessionBase(BaseModel):
    book_id: Optional[UUID] = None
    user_id: Optional[UUID] = None


class SessionCreate(SessionBase):
    pass


class SessionUpdate(BaseModel):
    user_id: Optional[UUID] = None


class SessionInDBBase(SessionBase):
    session_id: UUID = Field(default_factory=uuid4)
    created_at: datetime = Field(default_factory=datetime.now)
    updated_at: datetime = Field(default_factory=datetime.now)
    expires_at: Optional[datetime] = None


class Session(SessionInDBBase):
    session_id: UUID
    book_id: Optional[UUID] = None
    user_id: Optional[UUID] = None
    created_at: datetime
    updated_at: datetime
    expires_at: Optional[datetime]