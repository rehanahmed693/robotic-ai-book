from pydantic import BaseModel, ConfigDict
from typing import Optional
from uuid import UUID, uuid4
from datetime import datetime


class BaseSchema(BaseModel):
    """
    Base schema with common fields for all models
    """
    model_config = ConfigDict(from_attributes=True)
    
    id: Optional[UUID] = None
    created_at: Optional[datetime] = None
    updated_at: Optional[datetime] = None


class BaseChunkSchema(BaseModel):
    """
    Base schema for chunk-related models
    """
    chunk_id: Optional[UUID] = None
    content: str
    chunk_index: int
    metadata: Optional[dict] = {}


class BaseResponse(BaseModel):
    """
    Base response schema
    """
    success: bool = True
    message: Optional[str] = None
    timestamp: datetime = datetime.now()