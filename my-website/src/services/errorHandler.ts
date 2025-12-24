/**
 * Error handling utilities for API communication
 */

export interface ApiError {
  message: string;
  status?: number;
  details?: any;
}

class ApiErrorHandler {
  /**
   * Handle API errors from fetch responses
   */
  static async handleApiError(response: Response): Promise<ApiError> {
    let errorMessage = `API request failed with status ${response.status}: ${response.statusText}`;
    let errorDetails = null;

    try {
      // Attempt to parse error details from response
      const errorData = await response.json();
      if (errorData && errorData.detail) {
        errorMessage = errorData.detail;
        errorDetails = errorData;
      }
    } catch (e) {
      // If response is not JSON, use the status text as error message
      errorMessage = response.statusText;
    }

    return {
      message: errorMessage,
      status: response.status,
      details: errorDetails
    };
  }

  /**
   * Handle network errors
   */
  static handleNetworkError(error: any): ApiError {
    let message = 'Network error occurred';

    if (error instanceof TypeError && error.message.includes('fetch')) {
      message = 'Failed to connect to the server. Please check if the backend service is running.';
    } else if (error.message) {
      message = error.message;
    } else if (error.name === 'AbortError') {
      message = 'Request timed out. Please check your connection and try again.';
    }

    return {
      message,
      details: error
    };
  }

  /**
   * Format error for display to user
   */
  static formatUserErrorMessage(error: ApiError): string {
    if (error.status === 500) {
      return 'Server error occurred. The backend service encountered an issue. Please try again later.';
    } else if (error.status === 404) {
      return 'Requested resource not found. Please check if the backend API is properly configured.';
    } else if (error.status === 401) {
      return 'Authentication required. Please log in again.';
    } else if (error.status === 0) {
      return 'Failed to connect to the server. Please ensure the backend service is running and accessible.';
    } else if (error.message && error.message.includes('timeout')) {
      return 'Request timed out. Please check your network connection and try again.';
    } else if (error.message) {
      return error.message;
    } else {
      return 'An unexpected error occurred. Please try again.';
    }
  }
}

export default ApiErrorHandler;
export { ApiErrorHandler, ApiError };