import React from 'react';
import { ChatBotProvider } from '../contexts/ChatBotContext';

// This Root component wraps the entire app
export default function Root({ children }) {
  return <ChatBotProvider>{children}</ChatBotProvider>;
}