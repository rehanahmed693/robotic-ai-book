from typing import List
import logging
import openai
from ..core.config import settings


async def get_embeddings_openai(text: str) -> List[float]:
    """Get embeddings for a text using OpenAI API"""
    try:
        if not settings.openai_api_key:
            raise ValueError("OpenAI API key not provided in settings")
        
        openai.api_key = settings.openai_api_key
        
        response = await openai.Embedding.acreate(
            input=text,
            model="text-embedding-ada-002"
        )
        
        return response['data'][0]['embedding']
    except Exception as e:
        logging.error(f"Error getting embeddings from OpenAI: {e}")
        raise


async def get_embeddings_cohere(text: str) -> List[float]:
    """Get embeddings for a text using Cohere API"""
    try:
        import cohere
        
        if not settings.cohere_api_key:
            raise ValueError("Cohere API key not provided in settings")
        
        co = cohere.AsyncClient(settings.cohere_api_key)
        
        response = await co.embed(
            texts=[text],
            model='embed-english-v2.0'
        )
        
        return response.embeddings[0]
    except Exception as e:
        logging.error(f"Error getting embeddings from Cohere: {e}")
        raise


async def get_embeddings(text: str, use_cohere: bool = False) -> List[float]:
    """Get embeddings for a text using either OpenAI or Cohere API"""
    # If specifically requested to use Cohere or if OpenAI is not available but Cohere is
    if use_cohere or (not settings.openai_api_key and settings.cohere_api_key):
        return await get_embeddings_cohere(text)
    elif settings.openai_api_key:
        return await get_embeddings_openai(text)
    elif settings.cohere_api_key:
        # If no preference was specified but Cohere is available, use Cohere
        return await get_embeddings_cohere(text)
    else:
        raise ValueError("No embedding API key available")


def get_embedding_vector_size(use_cohere: bool = False) -> int:
    """Get the expected vector size based on the embedding model"""
    if use_cohere or (not settings.openai_api_key and settings.cohere_api_key):
        return 4096  # Cohere embedding size
    elif settings.openai_api_key:
        return 1536  # OpenAI embedding size
    elif settings.cohere_api_key:
        return 4096  # Cohere embedding size
    else:
        raise ValueError("No embedding API key available to determine vector size")