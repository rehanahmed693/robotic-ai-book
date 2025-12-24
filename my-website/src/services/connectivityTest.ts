/**
 * Quick test to verify connectivity works consistently
 */

import ApiService from './api';

// Test data
const testSessionRequest = {
  book_id: '00000000-0000-0000-0000-000000000000',
  user_id: '00000000-0000-0000-0000-000000000000',
};

export async function runConnectivityTest(): Promise<boolean> {
  console.log('Running connectivity test...');
  
  try {
    // Attempt to start a session
    console.log('1. Attempting to start a session...');
    const sessionResponse = await ApiService.startSession(testSessionRequest);
    console.log('✓ Session started successfully:', sessionResponse.session_id);
    
    // If session starts, try a simple query
    if (sessionResponse.session_id) {
      console.log('2. Attempting to submit a query...');
      const queryResponse = await ApiService.submitQuery({
        session_id: sessionResponse.session_id,
        question: 'Hello, are you there?',
        rag_mode: 'global',
      });
      console.log('✓ Query response received:', queryResponse.answer.substring(0, 50) + '...');

      // End the session
      console.log('3. Attempting to end the session...');
      const endResponse = await ApiService.endSession({
        session_id: sessionResponse.session_id
      });
      console.log('✓ Session ended successfully:', endResponse.message);
    }

    // Test the automatic session management endpoint
    console.log('4. Testing automatic session management endpoint...');
    const messageResponse = await ApiService.sendMessage({
      question: 'Hello, are you there with automatic session management?',
      rag_mode: 'global',
    });
    console.log('✓ Automatic session response received:', messageResponse.answer.substring(0, 50) + '...');
    
    console.log('✓ All connectivity tests passed!');
    return true;
  } catch (error) {
    console.error('✗ Connectivity test failed with error:', error);
    return false;
  }
}

// Run the test if this file is executed directly
if (typeof window !== 'undefined') {
  // In browser environment
  (window as any).runConnectivityTest = runConnectivityTest;
} else if (typeof global !== 'undefined') {
  // In Node.js environment
  (global as any).runConnectivityTest = runConnectivityTest;
}