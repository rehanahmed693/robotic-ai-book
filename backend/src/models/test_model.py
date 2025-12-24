from sqlalchemy import Column, Integer, String, DateTime, text
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
import enum
from datetime import datetime
from typing import Optional
from pydantic import BaseModel

Base = declarative_base()


class TestModel(Base):
    __tablename__ = "test_model"

    id = Column(Integer, primary_key=True, index=True)
    name = Column(String, index=True)
    description = Column(String, nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    def __repr__(self):
        return f"<TestModel(id={self.id}, name='{self.name}')>"


# Pydantic models for API
class TestModelBase(BaseModel):
    name: str
    description: Optional[str] = None


class TestModelCreate(TestModelBase):
    pass


class TestModelUpdate(TestModelBase):
    pass


class TestModelResponse(TestModelBase):
    id: int
    created_at: datetime
    updated_at: Optional[datetime] = None

    class Config:
        from_attributes = True