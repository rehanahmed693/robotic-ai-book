from typing import Optional, List
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, update, delete
from sqlalchemy.exc import SQLAlchemyError
from src.models.test_model import TestModel, TestModelCreate, TestModelUpdate
from fastapi import HTTPException
import logging

logger = logging.getLogger(__name__)


class TestModelService:
    def __init__(self, db_session: AsyncSession):
        self.db_session = db_session

    async def create(self, test_model: TestModelCreate) -> TestModel:
        """Create a new test model instance."""
        try:
            db_test_model = TestModel(**test_model.model_dump())
            self.db_session.add(db_test_model)
            await self.db_session.commit()
            await self.db_session.refresh(db_test_model)
            logger.info(f"Created TestModel with ID: {db_test_model.id}")
            return db_test_model
        except SQLAlchemyError as e:
            logger.error(f"Error creating TestModel: {e}")
            await self.db_session.rollback()
            raise HTTPException(status_code=500, detail=f"Database error: {str(e)}")

    async def get_by_id(self, test_model_id: int) -> Optional[TestModel]:
        """Get a test model by ID."""
        try:
            result = await self.db_session.execute(
                select(TestModel).where(TestModel.id == test_model_id)
            )
            test_model = result.scalar_one_or_none()
            return test_model
        except SQLAlchemyError as e:
            logger.error(f"Error retrieving TestModel with ID {test_model_id}: {e}")
            raise HTTPException(status_code=500, detail=f"Database error: {str(e)}")

    async def get_all(self, skip: int = 0, limit: int = 100) -> List[TestModel]:
        """Get all test models with pagination."""
        try:
            result = await self.db_session.execute(
                select(TestModel).offset(skip).limit(limit)
            )
            test_models = result.scalars().all()
            return test_models
        except SQLAlchemyError as e:
            logger.error(f"Error retrieving TestModels: {e}")
            raise HTTPException(status_code=500, detail=f"Database error: {str(e)}")

    async def update(self, test_model_id: int, test_model_update: TestModelUpdate) -> Optional[TestModel]:
        """Update a test model by ID."""
        try:
            # Check if the record exists
            existing_model = await self.get_by_id(test_model_id)
            if not existing_model:
                return None

            # Perform the update
            await self.db_session.execute(
                update(TestModel)
                .where(TestModel.id == test_model_id)
                .values(**test_model_update.model_dump(exclude_unset=True))
            )
            await self.db_session.commit()

            # Refresh and return the updated model
            updated_model = await self.get_by_id(test_model_id)
            logger.info(f"Updated TestModel with ID: {test_model_id}")
            return updated_model
        except SQLAlchemyError as e:
            logger.error(f"Error updating TestModel with ID {test_model_id}: {e}")
            await self.db_session.rollback()
            raise HTTPException(status_code=500, detail=f"Database error: {str(e)}")

    async def delete(self, test_model_id: int) -> bool:
        """Delete a test model by ID."""
        try:
            result = await self.db_session.execute(
                delete(TestModel).where(TestModel.id == test_model_id)
            )
            await self.db_session.commit()
            deleted = result.rowcount > 0
            if deleted:
                logger.info(f"Deleted TestModel with ID: {test_model_id}")
            return deleted
        except SQLAlchemyError as e:
            logger.error(f"Error deleting TestModel with ID {test_model_id}: {e}")
            await self.db_session.rollback()
            raise HTTPException(status_code=500, detail=f"Database error: {str(e)}")