export type RAGMode = 'full-book' | 'selection';

export interface ChatRequest {
  messages: Array<{
    role: 'user' | 'assistant' | 'system';
    content: string;
  }>;
  mode: RAGMode;
  context?: {
    pageUrl: string;
    selection?: string;
    chapterTitle?: string;
  };
}

export interface RetrievedChunk {
  content: string;
  metadata: {
    source_url: string;
    chapter_title: string;
    section_header: string;
    score: number;
  };
}
