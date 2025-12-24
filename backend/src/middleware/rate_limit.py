from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from typing import Dict
from collections import defaultdict
import time
import asyncio


class RateLimiter:
    def __init__(self, requests: int = 10, per_seconds: int = 60):
        self.requests = requests
        self.per_seconds = per_seconds
        self.requests_log: Dict[str, list] = defaultdict(list)
    
    def is_allowed(self, identifier: str) -> bool:
        now = time.time()
        # Clean old requests
        self.requests_log[identifier] = [
            req_time for req_time in self.requests_log[identifier]
            if now - req_time < self.per_seconds
        ]
        
        if len(self.requests_log[identifier]) >= self.requests:
            return False
        
        self.requests_log[identifier].append(now)
        return True


# Initialize rate limiter (10 requests per minute per IP)
rate_limiter = RateLimiter(requests=10, per_seconds=60)


async def rate_limit_middleware(request: Request, call_next):
    client_ip = request.client.host
    if not rate_limiter.is_allowed(client_ip):
        return JSONResponse(
            status_code=429,
            content={"detail": "Rate limit exceeded. Please try again later."}
        )
    
    response = await call_next(request)
    return response