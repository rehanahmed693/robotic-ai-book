from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import Response
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
import logging
from dotenv import load_dotenv

load_dotenv()
from . import chat_routes
from ..database.connections import db_connection


app = FastAPI(
    title="RAG Chatbot API",
    description="API for the integrated RAG chatbot that answers questions about book content",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "https://robotic-ai-book.vercel.app/","http://127.0.0.1:3000"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(chat_routes.router, prefix="/api/v1", tags=["chat"])

@app.get("/")
def read_root():
    return {"message": "Welcome to the RAG Chatbot API"}

@app.exception_handler(RequestValidationError)
async def validation_exception_handler(request, exc):
    """Custom handler for validation errors"""
    # Extract error details for more informative messages
    errors = []
    for error in exc.errors():
        field = " -> ".join(str(loc) for loc in error['loc'])
        message = error['msg']
        error_type = error['type']
        errors.append(f"Field '{field}': {message} (type: {error_type})")

    error_message = "Validation error(s): " + "; ".join(errors)
    return JSONResponse(
        status_code=422,
        content={"detail": error_message}
    )

@app.exception_handler(Exception)
async def general_exception_handler(request, exc):
    """General exception handler for unhandled exceptions"""
    logging.error(f"Unhandled exception: {exc}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={"detail": "Internal server error occurred. Please contact the administrator."}
    )

@app.on_event("startup")
async def startup_event():
    """Connect to the database when the application starts"""
    logging.info("Connecting to database...")
    try:
        await db_connection.connect()
        logging.info("Database connected successfully")
    except Exception as e:
        logging.error(f"Failed to connect to database: {e}")
        # For development purposes, we'll allow the app to start even without the database
        # In production, this would be a critical error
        logging.info("Continuing startup without database connection...")

@app.get('/favicon.ico', include_in_schema=False)
async def favicon():
    """Serve an empty response for favicon requests to prevent 404 errors"""
    return Response(status_code=204)


@app.on_event("shutdown")
async def shutdown_event():
    """Disconnect from the database when the application shuts down"""
    logging.info("Disconnecting from database...")
    await db_connection.disconnect()
    logging.info("Database disconnected successfully")