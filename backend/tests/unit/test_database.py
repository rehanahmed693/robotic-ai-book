import pytest
import os
from unittest.mock import patch, MagicMock
from sqlalchemy.ext.asyncio import create_async_engine
from src.config.database import get_db, engine, AsyncSessionLocal
from src.config.db_utils import test_connection, get_db_health, get_connection_string
from src.config.validation import validate_required_env_vars, validate_database_config, get_database_config


class TestDatabaseConfig:
    """Test cases for database configuration module."""

    def test_get_db_generator(self):
        """Test that get_db returns an async generator."""
        db_gen = get_db()
        assert hasattr(db_gen, '__aiter__')  # Check if it's an async generator

    @pytest.mark.asyncio
    async def test_test_connection_success(self):
        """Test that test_connection returns True when connection is successful."""
        # This test requires a real database connection to work properly
        # For now, we'll mock the engine.begin method
        with patch('src.config.db_utils.engine') as mock_engine:
            mock_conn = MagicMock()
            mock_conn.execute = MagicMock()
            mock_enter = MagicMock()
            mock_enter.__aenter__ = MagicMock(return_value=mock_conn)
            mock_engine.begin.return_value = mock_enter

            result = await test_connection()
            assert result is True

    @pytest.mark.asyncio
    async def test_test_connection_failure(self):
        """Test that test_connection returns False when connection fails."""
        with patch('src.config.db_utils.engine') as mock_engine:
            mock_conn = MagicMock()
            mock_conn.execute = MagicMock(side_effect=Exception("Connection failed"))
            mock_enter = MagicMock()
            mock_enter.__aenter__ = MagicMock(return_value=mock_conn)
            mock_engine.begin.return_value = mock_enter

            result = await test_connection()
            assert result is False

    def test_get_connection_string_with_url(self):
        """Test that get_connection_string returns a cleaned URL."""
        # Test with a sample URL containing credentials
        sample_url = "postgresql+asyncpg://user:pass@localhost:5432/dbname"
        
        with patch('src.config.db_utils.DATABASE_URL', sample_url):
            result = get_connection_string()
            # Should replace the password with ***
            assert "***" in result
            assert "user" in result  # Username should still be visible
            assert "localhost" in result  # Host should still be visible

    def test_get_connection_string_without_url(self):
        """Test that get_connection_string returns appropriate message when no URL is configured."""
        with patch('src.config.db_utils.DATABASE_URL', None):
            result = get_connection_string()
            assert result == "No database URL configured"


class TestValidation:
    """Test cases for validation functions."""

    def test_validate_required_env_vars_all_present(self):
        """Test that validation passes when all required variables are present."""
        # Mock environment variables
        with patch.dict(os.environ, {'TEST_VAR': 'value'}):
            result = validate_required_env_vars(['TEST_VAR'])
            assert result is True

    def test_validate_required_env_vars_missing(self):
        """Test that validation fails when required variables are missing."""
        # Ensure the variable is not in the environment
        if 'MISSING_VAR' in os.environ:
            del os.environ['MISSING_VAR']
        
        result = validate_required_env_vars(['MISSING_VAR'])
        assert result is False

    def test_get_database_config(self):
        """Test that get_database_config returns correct values."""
        with patch.dict(os.environ, {
            'NEON_DATABASE_URL': 'test_url',
            'DB_POOL_SIZE': '10',
            'DB_MAX_OVERFLOW': '20',
            'DB_POOL_TIMEOUT': '40.0',
            'DB_POOL_RECYCLE': '7200'
        }):
            config = get_database_config()
            assert config['database_url'] == 'test_url'
            assert config['pool_size'] == 10
            assert config['max_overflow'] == 20
            assert config['pool_timeout'] == 40.0
            assert config['pool_recycle'] == 7200

    def test_get_database_config_defaults(self):
        """Test that get_database_config returns default values when env vars are not set."""
        # Remove any existing config env vars
        env_backup = {}
        for key in ['DB_POOL_SIZE', 'DB_MAX_OVERFLOW', 'DB_POOL_TIMEOUT', 'DB_POOL_RECYCLE']:
            if key in os.environ:
                env_backup[key] = os.environ[key]
                del os.environ[key]
        
        try:
            config = get_database_config()
            assert config['pool_size'] == 5  # default
            assert config['max_overflow'] == 10  # default
            assert config['pool_timeout'] == 30.0  # default
            assert config['pool_recycle'] == 3600  # default
        finally:
            # Restore environment
            for key, value in env_backup.items():
                os.environ[key] = value