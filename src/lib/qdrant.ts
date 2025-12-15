import { QdrantClient } from '@qdrant/js-client-rest';
import { google } from '@ai-sdk/google';
import { embed } from 'ai';
import { RetrievedChunk, RAGMode } from './rag-types';

// Initialize clients
const qdrant = new QdrantClient({
  url: process.env.QDRANT_URL,
  apiKey: process.env.QDRANT_API_KEY,
});

const COLLECTION_NAME = 'textbook_v1';

export async function retrieveContext(
  query: string,
  mode: RAGMode,
  selection?: string
): Promise<RetrievedChunk[]> {
  // MODE B: Selected-Text Isolation
  if (mode === 'selection' && selection) {
    return [{
      content: selection,
      metadata: {
        source_url: 'user-selection',
        chapter_title: 'Current Context',
        section_header: 'User Selection',
        score: 1.0,
      }
    }];
  }

  // MODE A: Full-Book Retrieval
  // 1. Generate Embedding using Google Gemini
  const { embedding } = await embed({
    model: google.textEmbeddingModel('text-embedding-004'),
    value: query,
  });

  // 2. Query Qdrant
  const searchResult = await qdrant.search(COLLECTION_NAME, {
    vector: embedding,
    limit: 5,
    with_payload: true,
  });

  // 3. Transform
  return searchResult.map((hit) => ({
    content: hit.payload?.content as string || '',
    metadata: {
      source_url: hit.payload?.source_url as string || '',
      chapter_title: hit.payload?.chapter_title as string || '',
      section_header: hit.payload?.section_header as string || '',
      score: hit.score,
    },
  }));
}
