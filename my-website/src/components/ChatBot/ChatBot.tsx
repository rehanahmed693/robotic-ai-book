import React, { useState, useEffect, useRef } from 'react';
import { v4 as uuidv4 } from 'uuid';
import clsx from 'clsx';
import styles from './ChatBot.module.css';
import ApiService, {
  MessageRequest,
  QueryResponse,
  EndSessionRequest
} from '../../services/api';
import ApiErrorHandler from '../../services/errorHandler';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
}

interface ChatBotProps {
  className?: string;
}

const ChatBot: React.FC<ChatBotProps> = ({ className }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [ragMode, setRagMode] = useState<'global' | 'selection_only'>('global');
  const [currentSessionId, setCurrentSessionId] = useState<string | null>(null);

  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Add a welcome message when the chat opens
  useEffect(() => {
    if (isOpen && messages.length === 0) {
      setMessages([
        {
          id: 'welcome-' + Date.now(),
          role: 'assistant',
          content: 'Hello! I am your RAG chatbot. Ask me anything about the robotics and AI content.',
          timestamp: new Date(),
        }
      ]);
    }
  }, [isOpen, messages.length]);

  // Scroll to bottom when new messages arrive
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };


  // Reconnection logic when connection fails
  const attemptReconnection = async () => {
    setError(null); // Clear the error first
    // Add a fresh welcome message
    setMessages([
      {
        id: 'welcome-' + Date.now(),
        role: 'assistant',
        content: 'Hello! I am your RAG chatbot. Ask me anything about the robotics and AI content.',
        timestamp: new Date(),
      }
    ]);
  };

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Add user message to the chat
    const userMessage: Message = {
      id: 'user-' + Date.now(),
      role: 'user',
      content: inputValue,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      // Send message with automatic session management
      // The backend handles session creation and expiration automatically
      const request: MessageRequest = {
        question: userMessage.content,
        rag_mode: ragMode, // Use the selected RAG mode
        book_id: '123e4567-e89b-12d3-a456-426614174000', // Provide book ID for context
        session_id: currentSessionId || undefined // Pass current session ID if available
      };

      const data: QueryResponse = await ApiService.sendMessage(request);

      // Update the current session ID if returned from the backend
      if (data.session_id) {
        setCurrentSessionId(data.session_id);
      }

      // Add bot response to the chat
      const botMessage: Message = {
        id: 'bot-' + Date.now(),
        role: 'assistant',
        content: data.answer || 'Sorry, I could not process your request.',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (err: any) {
      console.error('Error sending message:', err);

      const errorMessage = ApiErrorHandler.formatUserErrorMessage({
        message: err.message || 'Unknown error occurred',
        status: err.status,
        details: err
      });

      setError(errorMessage);

      const errorBotMessage: Message = {
        id: 'error-' + Date.now(),
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorBotMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const toggleChat = () => {
    if (isOpen && currentSessionId) {
      // End the current session when closing the chat
      const endSessionRequest: EndSessionRequest = {
        session_id: currentSessionId
      };
      ApiService.endSession(endSessionRequest).catch(error => {
        console.error('Error ending session:', error);
        // Continue with closing the chat even if session ending fails
      });
      setCurrentSessionId(null);
    }
    setIsOpen(!isOpen);
  };

  return (
    <div className={clsx(styles.chatContainer, className)}>
      {isOpen ? (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>RAG Chatbot</h3>
            <button onClick={toggleChat} className={styles.closeButton}>
              ×
            </button>
          </div>

          {error && (
            <div className={styles.error}>
              {error}
              <button onClick={attemptReconnection} className={styles.retryButton}>
                Retry Connection
              </button>
            </div>
          )}

          <div className={styles.messagesArea}>
            <div className={styles.messagesContainer}>
              {messages.map((message) => (
                <div
                  key={message.id}
                  className={clsx(
                    styles.message,
                    styles[message.role]
                  )}
                >
                  <div className={styles.messageContent}>{message.content}</div>
                  <div className={styles.timestamp}>
                    {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </div>
                </div>
              ))}
              {isLoading && (
                <div className={clsx(styles.message, styles.assistant)}>
                  <div className={styles.typingIndicator}>
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              )}
              <div ref={messagesEndRef} />
            </div>
          </div>

          <div className={styles.inputArea}>
            <div className={styles.controlsRow}>
              <select
                value={ragMode}
                onChange={(e) => setRagMode(e.target.value as 'global' | 'selection_only')}
                className={styles.modeSelector}
                disabled={isLoading}
              >
                <option value="global">Global Search</option>
                <option value="selection_only">Selection Only</option>
              </select>
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyDown={handleKeyDown}
                placeholder="Ask a question about the content..."
                className={styles.textInput}
                disabled={isLoading}
              />
              <button
                onClick={sendMessage}
                className={clsx(styles.sendButton, isLoading && styles.disabled)}
                disabled={isLoading || !inputValue.trim()}
              >
                Send
              </button>
            </div>
          </div>
        </div>
      ) : (
        <button onClick={toggleChat} className={styles.floatingButton}>
          💬 Chat with RAG Bot
        </button>
      )}
    </div>
  );
};

export default ChatBot;