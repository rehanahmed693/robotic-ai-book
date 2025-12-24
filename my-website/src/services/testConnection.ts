/**
 * Test script to verify basic connectivity between frontend and backend
 */

import ApiService from './api';

// Test data
const testSessionRequest = {
  book_id: '00000000-0000-0000-0000-000000000000',
  user_id: '00000000-0000-0000-0000-000000000000',
};

async function testConnection() {
  console.log('Testing connection to the backend API...');
  
  try {
    // Attempt to start a session
    console.log('Attempting to start a session...');
    const sessionResponse = await ApiService.startSession(testSessionRequest);
    console.log('Session started successfully:', sessionResponse);
    
    // If session starts, try a simple query
    if (sessionResponse.session_id) {
      console.log('Attempting to submit a query...');
      const queryResponse = await ApiService.submitQuery({
        session_id: sessionResponse.session_id,
        question: 'Hello, are you there?',
        rag_mode: 'global',
      });
      console.log('Query response received:', queryResponse);
    }

    // Also test the automatic session management endpoint
    console.log('Testing automatic session management endpoint...');
    const messageResponse = await ApiService.sendMessage({
      question: 'Hello, are you there with automatic session management?',
      rag_mode: 'global',
    });
    console.log('Automatic session response received:', messageResponse);
  } catch (error) {
    console.error('Test failed with error:', error);
    return false;
  }
  
  console.log('Connection test completed successfully!');
  return true;
}

export default testConnection;