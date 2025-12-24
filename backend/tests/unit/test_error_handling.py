import pytest
from unittest.mock import patch, MagicMock
from fastapi.testclient import TestClient
from src.main import app
from src.config.db_utils import test_connection


class TestErrorHandling:
    """Test cases for error handling scenarios."""

    def test_health_endpoint_with_db_error(self):
        """Test that the health endpoint handles database errors gracefully."""
        # Mock the database session to raise an exception
        with patch('src.main.AsyncSessionLocal') as mock_session_local:
            mock_session = MagicMock()
            mock_session.__aenter__ = MagicMock(side_effect=Exception("Database connection failed"))
            mock_session.__aexit__ = MagicMock(return_value=None)
            mock_session_local.return_value = mock_session
            
            client = TestClient(app)
            response = client.get("/health")
            
            # Should return 200 with unhealthy status, not a 500 error
            assert response.status_code == 200
            data = response.json()
            assert data["status"] == "unhealthy"
            assert data["database"] == "disconnected"
            assert "error" in data

    @pytest.mark.asyncio
    async def test_db_connection_utility_with_error(self):
        """Test that the database connection utility handles errors gracefully."""
        with patch('src.config.db_utils.engine') as mock_engine:
            mock_conn = MagicMock()
            mock_conn.execute = MagicMock(side_effect=Exception("Connection failed"))
            mock_enter = MagicMock()
            mock_enter.__aenter__ = MagicMock(return_value=mock_conn)
            mock_engine.begin.return_value = mock_enter

            result = await test_connection()
            # Should return False when connection fails, not raise an exception
            assert result is False

    def test_startup_event_with_db_error(self):
        """Test that the startup event handles database errors gracefully."""
        # This test is more complex as it involves the startup event
        # We'll test by mocking the test_connection function
        with patch('src.main.test_connection') as mock_test_connection:
            mock_test_connection.side_effect = Exception("Startup DB error")
            
            # Create a new TestClient to trigger the startup event
            client = TestClient(app)
            
            # The app should still start even if DB connection fails
            response = client.get("/")
            assert response.status_code == 200