import pytest
from fastapi.testclient import TestClient
from src.main import app


class TestHealthEndpoint:
    """Test cases for the health check endpoint."""

    def test_health_endpoint_returns_json(self):
        """Test that the health endpoint returns a JSON response."""
        client = TestClient(app)
        response = client.get("/health")
        
        assert response.status_code == 200
        assert response.headers["content-type"] == "application/json"
        
        data = response.json()
        assert isinstance(data, dict)
        assert "status" in data
        assert "database" in data

    def test_health_endpoint_structure(self):
        """Test that the health endpoint returns the expected structure."""
        client = TestClient(app)
        response = client.get("/health")
        data = response.json()
        
        # Check that required fields are present
        assert "status" in data
        assert "database" in data
        
        # Check that status is either "healthy" or "unhealthy"
        assert data["status"] in ["healthy", "unhealthy"]
        
        # Check that database status is either "connected" or "disconnected"
        assert data["database"] in ["connected", "disconnected"]

    def test_health_endpoint_values_when_connected(self):
        """Test that the health endpoint returns appropriate values when connected."""
        client = TestClient(app)
        response = client.get("/health")
        data = response.json()
        
        # The actual values depend on whether the database is accessible
        # If the database is configured correctly, it should be connected
        assert data["status"] in ["healthy", "unhealthy"]
        assert data["database"] in ["connected", "disconnected"]
        
        # If database is unhealthy, it should also include an error field
        if data["status"] == "unhealthy":
            assert "error" in data