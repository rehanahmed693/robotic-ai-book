import os
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker
from dotenv import load_dotenv
from urllib.parse import urlparse

# Load environment variables from .env file
load_dotenv()

# Get database URL from environment variables
DATABASE_URL = os.getenv("NEON_DATABASE_URL")  # Using the variable name from the existing .env

if not DATABASE_URL:
    raise ValueError("NEON_DATABASE_URL environment variable not set")

# Parse the database URL to check if it's a Neon connection
parsed_url = urlparse(DATABASE_URL)
is_neon_db = 'neon.tech' in parsed_url.hostname if parsed_url.hostname else False

# Configure engine with SSL settings for Neon
connect_args = {}
if is_neon_db:
    connect_args = {
        "ssl": "require",  # Require SSL for Neon connections
    }

# Create async engine
engine = create_async_engine(
    DATABASE_URL,
    echo=False,  # Set to True to log SQL queries
    pool_size=5,
    max_overflow=10,
    pool_timeout=30.0,
    pool_recycle=300,  # Shorter recycle time for Neon
    pool_pre_ping=True,
    connect_args=connect_args  # Add SSL settings for Neon
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