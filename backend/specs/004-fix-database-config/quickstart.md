# Quickstart: Database Configuration for FastAPI Backend

## Prerequisites

- Python 3.11 or higher
- PostgreSQL database server
- pip package manager

## Setup Instructions

### 1. Install Dependencies

```bash
pip install fastapi uvicorn sqlalchemy asyncpg python-dotenv psycopg2-binary
```

### 2. Create Environment File

Create a `.env` file in the project root with your database configuration:

```env
DATABASE_URL=postgresql+asyncpg://username:password@localhost:5432/database_name
```

Replace `username`, `password`, `localhost`, `5432`, and `database_name` with your actual PostgreSQL connection details.

### 3. Database Configuration Module

Create a database configuration module at `src/config/database.py`:

```python
import os
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Get database URL from environment variables
DATABASE_URL = os.getenv("DATABASE_URL")

if not DATABASE_URL:
    raise ValueError("DATABASE_URL environment variable not set")

# Create async engine
engine = create_async_engine(
    DATABASE_URL,
    echo=False,  # Set to True to log SQL queries
    pool_size=5,
    max_overflow=10,
    pool_timeout=30.0,
    pool_recycle=3600,
    pool_pre_ping=True
)

# Create async session maker
AsyncSessionLocal = sessionmaker(
    engine, 
    class_=AsyncSession, 
    expire_on_commit=False
)

# Dependency to get database session
async def get_db():
    async with AsyncSessionLocal() as session:
        yield session
```

### 4. Update Main Application

Update your main application file (e.g., `src/main.py`) to include database initialization:

```python
from fastapi import FastAPI
from src.config.database import engine
import asyncio

app = FastAPI()

# Import your models here to ensure they're registered with SQLAlchemy
# from src.models import *  # Import your models

@app.on_event("startup")
async def startup_event():
    print("Connecting to database...")
    # Optionally, test the connection
    try:
        async with engine.begin() as conn:
            # This will raise an exception if the connection fails
            await conn.execute("SELECT 1")
        print("Database connected successfully!")
    except Exception as e:
        print(f"Database connection failed: {e}")

@app.on_event("shutdown")
async def shutdown_event():
    print("Closing database connection...")
    await engine.dispose()
    print("Database connection closed!")

@app.get("/")
def read_root():
    return {"Hello": "World"}
```

### 5. Verify the Setup

Run your FastAPI application:

```bash
uvicorn src.main:app --reload
```

You should no longer see the "WARNING: No database URL provided" message. The application will connect to PostgreSQL during startup.

### 6. Testing the Connection

You can add a health check endpoint to verify the database connection:

```python
@app.get("/health")
async def health_check():
    try:
        async with AsyncSessionLocal() as session:
            await session.execute("SELECT 1")
        return {"status": "healthy", "database": "connected"}
    except Exception as e:
        return {"status": "unhealthy", "database": "disconnected", "error": str(e)}
```

## Troubleshooting

### Common Issues:

1. **"No database URL provided" error**: Ensure your `.env` file exists and contains the `DATABASE_URL` variable.

2. **Connection refused**: Verify that PostgreSQL is running and accessible at the specified host and port.

3. **Authentication failed**: Check that your username and password are correct.

4. **SSL connection error**: If your PostgreSQL server requires SSL, update your connection string accordingly.

### Environment Variables:

- `DATABASE_URL`: Full PostgreSQL connection string (required)
- Optional: Additional connection parameters can be added to the DATABASE_URL string