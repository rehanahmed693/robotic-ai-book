/**
 * API Service for handling communication with the backend
 * Provides methods for chat functionality
 */

import ApiErrorHandler from './errorHandler';
import ConnectionLogger from './connectionLogger';

// Base URL for the backend API
// Using a global variable or default value as process.env is not available in browser
const API_BASE_URL = ((): string => {
  // Check for environment variable in a browser-safe way
  if (typeof process !== 'undefined' && process?.env?.REACT_APP_API_BASE_URL) {
    return process.env.REACT_APP_API_BASE_URL;
  }

  // Check for global environment variable (set by Docusaurus or other build tools)
  if (typeof window !== 'undefined' && (window as any)._env_?.REACT_APP_API_BASE_URL) {
    return (window as any)._env_.REACT_APP_API_BASE_URL;
  }

  // Default fallback
  return 'https://rehanbackend-rag-bot.hf.space';
})();

const DEFAULT_TIMEOUT = 20000; // 10 seconds

interface StartSessionRequest {
  book_id: string;
  user_id: string;
}

interface StartSessionResponse {
  session_id: string;
  message: string;
}

// Define the allowed RAG modes to match backend enum
type RagMode = 'global' | 'selection_only';

interface QueryRequest {
  session_id?: string;  // Optional for explicit session management (used by /api/v1/chat/query)
  question?: string;
  message?: string;
  rag_mode?: RagMode;
  selection_context?: string;
  book_id?: string;
}

interface QueryResponse {
  answer: string;
  citations: any[];
  confidence_score: number;
  session_id?: string;
}

interface GetHistoryRequest {
  session_id: string;
  limit?: number;
  offset?: number;
}

interface GetHistoryResponse {
  history: any[];
}

interface EndSessionRequest {
  session_id: string;
}

interface EndSessionResponse {
  message: string;
}

// Timeout function to wrap fetch requests
const fetchWithTimeout = async (url: string, options: RequestInit, timeout: number = DEFAULT_TIMEOUT) => {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), timeout);

  try {
    const response = await fetch(url, {
      ...options,
      signal: controller.signal,
    });
    clearTimeout(timeoutId);
    return response;
  } catch (error) {
    clearTimeout(timeoutId);
    throw error;
  }
};

class ApiService {
  /**
   * Start a new chat session
   */
  static async startSession(request: StartSessionRequest): Promise<StartSessionResponse> {
    ConnectionLogger.log('request', `Starting new session: ${JSON.stringify(request)}`);

    try {
      // Prepare the request body ensuring proper format
      const requestBody = {
        book_id: request.book_id,
        user_id: request.user_id || undefined
      };

      const response = await fetchWithTimeout(`${API_BASE_URL}/api/v1/chat/start`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        const error = await ApiErrorHandler.handleApiError(response);
        ConnectionLogger.log('error', `Failed to start session: ${error.message}`);
        throw new Error(error.message);
      }

      const result = await response.json();
      ConnectionLogger.log('response', `Session started successfully: ${result.session_id}`);
      return result;
    } catch (error: any) {
      console.error('Error starting session:', error);
      ConnectionLogger.log('error', `Error starting session: ${error.message || error}`);

      if (error.message) {
        throw error;
      } else {
        const networkError = ApiErrorHandler.handleNetworkError(error);
        throw new Error(networkError.message);
      }
    }
  }

  /**
   * Submit a query to the chat endpoint
   */
  static async submitQuery(request: QueryRequest): Promise<QueryResponse> {
    ConnectionLogger.log('request', `Submitting query for session ${request.session_id}`);

    try {
      // Prepare the request body ensuring proper format
      const requestBody = {
        session_id: request.session_id || undefined,
        question: request.question || request.message || undefined,
        rag_mode: request.rag_mode || 'global',
        selection_context: request.selection_context || undefined,
        book_id: request.book_id || undefined
      };

      const response = await fetchWithTimeout(`${API_BASE_URL}/api/v1/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        const error = await ApiErrorHandler.handleApiError(response);
        ConnectionLogger.log('error', `Failed to submit query: ${error.message}`);
        throw new Error(error.message);
      }

      const result = await response.json();
      ConnectionLogger.log('response', `Query response received for session ${request.session_id}`);
      return result;
    } catch (error: any) {
      console.error('Error submitting query:', error);
      ConnectionLogger.log('error', `Error submitting query: ${error.message || error}`);

      if (error.message) {
        throw error;
      } else {
        const networkError = ApiErrorHandler.handleNetworkError(error);
        throw new Error(networkError.message);
      }
    }
  }

  /**
   * Send a message to the chat endpoint (mapped to /api/v1/chat/query)
   */
  static async sendMessage(request: QueryRequest): Promise<QueryResponse> {
    ConnectionLogger.log('request', `Sending message for session ${request.session_id}`);

    try {
      // Prepare the request body ensuring proper format
      const requestBody = {
        session_id: request.session_id || undefined,
        question: request.question || request.message || undefined,
        rag_mode: request.rag_mode || 'global',
        selection_context: request.selection_context || undefined,
        book_id: request.book_id || undefined
      };

      const response = await fetchWithTimeout(`${API_BASE_URL}/api/v1/chat/query`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        const error = await ApiErrorHandler.handleApiError(response);
        ConnectionLogger.log('error', `Failed to send message: ${error.message}`);
        throw new Error(error.message);
      }

      const result = await response.json();
      ConnectionLogger.log('response', `Message response received for session ${request.session_id}`);
      return result;
    } catch (error: any) {
      console.error('Error sending message:', error);
      ConnectionLogger.log('error', `Error sending message: ${error.message || error}`);

      if (error.message) {
        throw error;
      } else {
        const networkError = ApiErrorHandler.handleNetworkError(error);
        throw new Error(networkError.message);
      }
    }
  }

  /**
   * Get conversation history
   */
  static async getConversationHistory(request: GetHistoryRequest): Promise<GetHistoryResponse> {
    ConnectionLogger.log('request', `Getting conversation history for session ${request.session_id}`);

    try {
      // Construct URL with query parameters
      const url = new URL(`${API_BASE_URL}/api/v1/chat/history`);
      url.searchParams.append('session_id', request.session_id);
      if (request.limit !== undefined) {
        url.searchParams.append('limit', request.limit.toString());
      }
      if (request.offset !== undefined) {
        url.searchParams.append('offset', request.offset.toString());
      }

      const response = await fetchWithTimeout(url.toString(), {
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        const error = await ApiErrorHandler.handleApiError(response);
        ConnectionLogger.log('error', `Failed to get conversation history: ${error.message}`);
        throw new Error(error.message);
      }

      const result = await response.json();
      ConnectionLogger.log('response', `Conversation history received for session ${request.session_id}`);
      return result;
    } catch (error: any) {
      console.error('Error getting conversation history:', error);
      ConnectionLogger.log('error', `Error getting conversation history: ${error.message || error}`);

      if (error.message) {
        throw error;
      } else {
        const networkError = ApiErrorHandler.handleNetworkError(error);
        throw new Error(networkError.message);
      }
    }
  }

  /**
   * End a chat session
   */
  static async endSession(request: EndSessionRequest): Promise<EndSessionResponse> {
    ConnectionLogger.log('request', `Ending session: ${request.session_id}`);

    try {
      // Prepare the request body ensuring proper format
      const requestBody = {
        session_id: request.session_id
      };

      const response = await fetchWithTimeout(`${API_BASE_URL}/api/v1/chat/end`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        const error = await ApiErrorHandler.handleApiError(response);
        ConnectionLogger.log('error', `Failed to end session: ${error.message}`);
        throw new Error(error.message);
      }

      const result = await response.json();
      ConnectionLogger.log('response', `Session ended successfully: ${request.session_id}`);
      return result;
    } catch (error: any) {
      console.error('Error ending session:', error);
      ConnectionLogger.log('error', `Error ending session: ${error.message || error}`);

      if (error.message) {
        throw error;
      } else {
        const networkError = ApiErrorHandler.handleNetworkError(error);
        throw new Error(networkError.message);
      }
    }
  }
}

export default ApiService;