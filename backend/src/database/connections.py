import asyncio
import sys
from typing import Optional
import psycopg_pool
from urllib.parse import urlparse
from ..core.config import settings
import logging


# Fix for Windows compatibility
if sys.platform.startswith('win'):
    import asyncio
    try:
        from asyncio import WindowsSelectorEventLoopPolicy
    except ImportError:
        pass
    else:
        if not isinstance(asyncio.get_event_loop_policy(), WindowsSelectorEventLoopPolicy):
            asyncio.set_event_loop_policy(WindowsSelectorEventLoopPolicy())


class DatabaseConnection:
    def __init__(self):
        self.pool: Optional[psycopg_pool.AsyncConnectionPool] = None
        self.is_connected = False

    async def connect(self):
        """Initialize the database connection pool"""
        if not settings.neon_database_url:
            logging.warning("No database URL provided. Database operations will fail until a valid PostgreSQL connection is configured.")
            return

        try:
            # Parse the database URL to validate it's a PostgreSQL URL
            parsed_url = urlparse(settings.neon_database_url)
            if parsed_url.scheme not in ('postgresql', 'postgres'):
                raise ValueError("Only PostgreSQL URLs are supported")

            # Configure connection pool with proper SSL settings for Neon
            # Using only valid parameters for AsyncConnectionPool
            self.pool = psycopg_pool.AsyncConnectionPool(
                conninfo=settings.neon_database_url,
                min_size=2,          # Reduced minimum size
                max_size=10,         # Reduced maximum size
                max_waiting=10,      # Max requests waiting for a connection
                max_idle=30,         # Close idle connections after 30 seconds
                max_lifetime=60,     # Recreate connections after 60 seconds
                timeout=60,          # Extended timeout for SSL handshake
            )

            # Wait for the pool to be ready
            await self.pool.open()

            # Verify connection by trying to acquire a connection
            conn = await self.pool.getconn()
            try:
                # Test the connection with a simple query
                cur = await conn.execute("SELECT 1")
                await cur.fetchall()
                self.is_connected = True
                logging.info("Database connected successfully")
            finally:
                await self.pool.putconn(conn)

        except Exception as e:
            logging.error(f"Failed to connect to database: {e}")
            self.is_connected = False
            # Re-raise so the application startup fails appropriately
            raise

    async def disconnect(self):
        """Close the database connection pool"""
        if self.pool:
            await self.pool.close()
            self.is_connected = False

    async def get_connection(self):
        """Get a connection from the pool"""
        if not self.is_connected:
            raise RuntimeError("Database not connected. Call connect() first or verify the database server is running.")

        try:
            return await self.pool.getconn()
        except Exception as e:
            logging.error(f"Error getting connection from pool: {e}")
            # Attempt to reconnect if connection failed
            await self.connect()
            return await self.pool.getconn()

    async def release_connection(self, conn):
        """Release a connection back to the pool"""
        if self.pool:
            try:
                # Check if connection is still valid before releasing it back to the pool
                if conn.closed:
                    # If the connection is closed, discard it and create a new one
                    logging.warning("Discarded closed connection")
                else:
                    await self.pool.putconn(conn)
            except Exception as e:
                logging.error(f"Error releasing connection to pool: {e}")
                # If putting connection back fails, close it to free resources
                try:
                    if not conn.closed:
                        await conn.close()
                except:
                    pass  # Ignore errors when closing


# Global database connection instance
db_connection = DatabaseConnection()