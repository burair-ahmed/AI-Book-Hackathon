import React, { useState, useEffect, useRef } from 'react';
// import { useChat } from '@ai-sdk/react'; // Removed manual fetch fallback
import clsx from 'clsx';
import { useLocation } from '@docusaurus/router';
import styles from './styles.module.css';

interface ChatWidgetProps {
  defaultOpen?: boolean;
}

export default function ChatWidget({ defaultOpen = false }: ChatWidgetProps) {
  const [isOpen, setIsOpen] = useState(defaultOpen);
  const [selection, setSelection] = useState<string | null>(null);
  const location = useLocation();
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const [messages, setMessages] = useState<any[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [inputValue, setInputValue] = useState('');

  const appendMessage = (role: 'user' | 'assistant', content: string) => {
    setMessages(prev => [...prev, { id: crypto.randomUUID(), role, content }]);
  };

  const safeSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userContent = inputValue;
    setInputValue('');
    appendMessage('user', userContent);
    setIsLoading(true);

    try {
      const response = await fetch('/api/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          messages: [...messages, { role: 'user', content: userContent }],
          mode: selection ? 'selection' : 'full-book',
          context: {
            pageUrl: location.pathname,
            selection: selection || undefined,
          },
        }),
      });

      if (!response.ok) throw new Error(response.statusText);
      if (!response.body) throw new Error('No response body');

      // Create a placeholder for the assistant's reply
      const assistantId = crypto.randomUUID();
      setMessages(prev => [...prev, { id: assistantId, role: 'assistant', content: '' }]);

      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      let done = false;
      let accumulatedContent = '';

      while (!done) {
        const { value, done: readerDone } = await reader.read();
        done = readerDone;
        if (value) {
          const chunk = decoder.decode(value, { stream: true });
          accumulatedContent += chunk;
          
          setMessages(prev => 
            prev.map(m => 
              m.id === assistantId ? { ...m, content: accumulatedContent } : m
            )
          );
        }
      }
    } catch (error: any) {
      console.error('Chat Error:', error);
      appendMessage('assistant', `Error: ${error.message || 'Failed to fetch response.'}`);
    } finally {
      setIsLoading(false);
    }
  };



  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Handle Text Selection
  useEffect(() => {
    const handleSelection = () => {
      const text = window.getSelection()?.toString().trim();
      if (text && text.length > 5) {
        // Only update selection, don't auto-open (UX choice)
        // Optionally show a "Ask about this" tooltip here
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  // Expose a helper to set selection from outside (e.g. tooltip)
  useEffect(() => {
    // @ts-ignore
    window.setChatSelection = (text: string) => {
      setSelection(text);
      setIsOpen(true);
    };
  }, []);

  const clearSelection = () => setSelection(null);

  return (
    <div className={styles.chatContainer}>
      <div className={clsx(styles.window, !isOpen && styles.hidden)}>
        <div className={styles.header}>
          <span>AI Assistant</span>
          <button onClick={() => setIsOpen(false)} style={{background:'none', border:'none', color:'white', cursor:'pointer'}}>✕</button>
        </div>
        
        {selection && (
          <div className={styles.selectionBanner}>
            <span>Targeting selection ({selection.substring(0, 30)}...)</span>
            <button onClick={clearSelection} className={styles.clearSelection}>×</button>
          </div>
        )}

        <div className={styles.messages}>
            {messages.length === 0 && (
                <div style={{opacity: 0.6, fontSize: '0.9rem', textAlign: 'center', marginTop: '1rem'}}>
                    Ask me anything about the textbook.<br/>
                    Select text to ask specific questions.
                </div>
            )}
          {messages.map((m: any) => (
            <div key={m.id} className={clsx(styles.message, m.role === 'user' ? styles.user : styles.assistant)}>
              {m.role === 'user' ? '👤 ' : '🤖 '}
              {m.content}
            </div>
          ))}
          {isLoading && <div className={styles.message} style={{opacity: 0.5}}>Thinking...</div>}
          <div ref={messagesEndRef} />
        </div>

        <div className={styles.inputArea}>
          <form onSubmit={safeSubmit} className={styles.form}>
            <input
              className={styles.input}
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder={selection ? "Ask about selection..." : "Ask the book..."}
            />
            <button type="submit" className={styles.sendButton} disabled={isLoading}>
              Send
            </button>
          </form>
        </div>
      </div>

      <button className={styles.toggleButton} onClick={() => setIsOpen(!isOpen)}>
        {isOpen ? '↓' : '💬'}
      </button>
    </div>
  );
}
