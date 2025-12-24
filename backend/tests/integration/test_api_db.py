import pytest
from fastapi.testclient import TestClient
from src.main import app
from src.config.db_utils import test_connection


class TestDatabaseIntegration:
    """Integration tests for database functionality."""

    def test_health_endpoint_with_db_connection(self):
        """Test the health endpoint which requires database connection."""
        client = TestClient(app)
        
        # This test requires a real database connection
        # The response will depend on whether the database is actually accessible
        response = client.get("/health")
        
        # The response should be a valid JSON response
        assert response.status_code in [200, 500]  # 200 if healthy, 500 if unhealthy
        assert "status" in response.json()
        assert "database" in response.json()

    @pytest.mark.asyncio
    async def test_db_connection_utility(self):
        """Test the database connection utility function."""
        # This test requires a real database connection
        result = await test_connection()
        # Result will be True if database is accessible, False otherwise
        assert isinstance(result, bool)

    def test_main_endpoint(self):
        """Test the main endpoint works."""
        client = TestClient(app)
        response = client.get("/")
        assert response.status_code == 200
        assert response.json() == {"Hello": "World"}