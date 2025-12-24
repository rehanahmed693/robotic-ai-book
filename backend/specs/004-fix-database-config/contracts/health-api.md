# Database Health Check API Contract

## Overview
This API contract defines the health check endpoint that verifies the database connection status in the FastAPI backend.

## Endpoint: GET /health

### Description
Checks the health status of the application, including database connectivity.

### Request
- **Method**: GET
- **Path**: `/health`
- **Headers**: None required
- **Parameters**: None
- **Body**: None

### Response
- **Success Response**:
  - **Code**: 200 OK
  - **Content-Type**: application/json
  - **Body**:
    ```json
    {
      "status": "healthy",
      "database": "connected"
    }
    ```

- **Error Response**:
  - **Code**: 200 OK (still returns 200 but indicates unhealthy state)
  - **Content-Type**: application/json
  - **Body**:
    ```json
    {
      "status": "unhealthy",
      "database": "disconnected",
      "error": "Detailed error message"
    }
    ```

### Example Request
```
GET /health
```

### Example Response (Healthy)
```
Status: 200 OK
Content-Type: application/json

{
  "status": "healthy",
  "database": "connected"
}
```

### Example Response (Unhealthy)
```
Status: 200 OK
Content-Type: application/json

{
  "status": "unhealthy",
  "database": "disconnected",
  "error": "FATAL: password authentication failed for user \"invalid_user\""
}
```

## Purpose
This endpoint allows external services and monitoring tools to verify that the application is running and can successfully communicate with the database. It helps identify when the database connection issue has been resolved.