import React, { createContext, useContext, useState, ReactNode } from 'react';
import ChatBot from '../components/ChatBot/ChatBot';

interface ChatBotContextType {
  showChatBot: boolean;
  setShowChatBot: (show: boolean) => void;
}

const ChatBotContext = createContext<ChatBotContextType | undefined>(undefined);

export const ChatBotProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
  const [showChatBot, setShowChatBot] = useState(true);

  return (
    <ChatBotContext.Provider value={{ showChatBot, setShowChatBot }}>
      {children}
      {showChatBot && <ChatBot />}
    </ChatBotContext.Provider>
  );
};

export const useChatBot = (): ChatBotContextType => {
  const context = useContext(ChatBotContext);
  if (!context) {
    throw new Error('useChatBot must be used within a ChatBotProvider');
  }
  return context;
};