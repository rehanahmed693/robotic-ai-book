import os
from typing import List
import logging

logger = logging.getLogger(__name__)


def validate_required_env_vars(required_vars: List[str]) -> bool:
    """
    Validate that all required environment variables are set.
    
    Args:
        required_vars: List of required environment variable names
        
    Returns:
        True if all required variables are set, False otherwise
    """
    missing_vars = []
    
    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)
    
    if missing_vars:
        logger.error(f"Missing required environment variables: {', '.join(missing_vars)}")
        return False
    
    logger.info(f"All required environment variables are set: {', '.join(required_vars)}")
    return True


def get_database_config():
    """
    Get database configuration from environment variables.
    
    Returns:
        Dictionary containing database configuration
    """
    return {
        "database_url": os.getenv("NEON_DATABASE_URL"),
        "pool_size": int(os.getenv("DB_POOL_SIZE", "5")),
        "max_overflow": int(os.getenv("DB_MAX_OVERFLOW", "10")),
        "pool_timeout": float(os.getenv("DB_POOL_TIMEOUT", "30.0")),
        "pool_recycle": int(os.getenv("DB_POOL_RECYCLE", "3600")),
    }


def validate_database_config():
    """
    Validate the database configuration.
    
    Returns:
        True if configuration is valid, False otherwise
    """
    config = get_database_config()
    
    # Check if database URL is provided
    if not config["database_url"]:
        logger.error("DATABASE_URL environment variable is not set")
        return False
    
    # Check if pool parameters are valid
    if config["pool_size"] <= 0:
        logger.error("DB_POOL_SIZE must be greater than 0")
        return False
    
    if config["max_overflow"] < 0:
        logger.error("DB_MAX_OVERFLOW must be greater than or equal to 0")
        return False
    
    if config["pool_timeout"] <= 0:
        logger.error("DB_POOL_TIMEOUT must be greater than 0")
        return False
    
    if config["pool_recycle"] < 0:
        logger.error("DB_POOL_RECYCLE must be greater than or equal to 0")
        return False
    
    logger.info("Database configuration is valid")
    return True