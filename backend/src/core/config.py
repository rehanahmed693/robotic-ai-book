from pydantic_settings import BaseSettings, SettingsConfigDict
from typing import Optional
import os
from dotenv import load_dotenv
import logging

load_dotenv()

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class Settings(BaseSettings):
    # Database settings
    qdrant_url: str = ""
    qdrant_api_key: Optional[str] = None
    neon_database_url: str = ""

    # API settings
    openai_api_key: Optional[str] = os.getenv("OPENAI_API_KEY")  # Load from environment variable
    cohere_api_key: Optional[str] = os.getenv("COHERE_API_KEY")  # Load from environment variable
    google_api_key: Optional[str] = os.getenv("GOOGLE_API_KEY")  # Load from environment variable

    # Application settings
    debug: bool = os.getenv("DEBUG", "False").lower() == "true"
    app_name: str = "RAG Chatbot API"
    version: str = "1.0.0"

    model_config = SettingsConfigDict(
        env_file=".env",
        case_sensitive=False,
        extra="ignore"  # This ignores extra environment variables
    )

    def model_post_init(self, __context):
        """Validate settings after initialization"""
        if not self.neon_database_url:
            logger.warning("NEON_DATABASE_URL is not set. Database operations will fail until a valid PostgreSQL connection is configured.")
        
        if not self.qdrant_url:
            logger.warning("QDRANT_URL is not set. Vector store operations will fail until a valid Qdrant connection is configured.")


# Create a single instance of settings
settings = Settings()