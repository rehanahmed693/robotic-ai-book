import asyncio
from sqlalchemy import text
from .database import engine


async def test_connection():
    """
    Test the database connection by executing a simple query.
    Returns True if connection is successful, False otherwise.
    """
    try:
        async with engine.begin() as conn:
            await conn.execute(text("SELECT 1"))
        return True
    except Exception as e:
        print(f"Database connection test failed: {e}")
        return False


async def get_db_health():
    """
    Get detailed health information about the database connection.
    """
    try:
        async with engine.begin() as conn:
            # Get basic connection info
            result = await conn.execute(text("SELECT version()"))
            db_version = result.scalar()
            
            # Get connection pool info (if available)
            pool_info = {
                "pool_size": engine.pool.size(),
                "checkedin": engine.pool.checkedin(),
                "overflow": engine.pool.overflow(),
            }
            
            return {
                "status": "healthy",
                "database_version": db_version,
                "pool_info": pool_info
            }
    except Exception as e:
        return {
            "status": "unhealthy",
            "error": str(e)
        }


def get_connection_string():
    """
    Get the connection string without sensitive information for logging purposes.
    """
    from .database import DATABASE_URL
    if DATABASE_URL:
        # Remove sensitive information from the URL for logging
        import re
        clean_url = re.sub(r':[^@]*@', ':***@', DATABASE_URL)
        return clean_url
    return "No database URL configured"