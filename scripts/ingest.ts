import fs from 'fs';
import path from 'path';
import { glob } from 'glob';
import { parseAndChunk } from './lib/chunker';
import { QdrantClient } from '@qdrant/js-client-rest';
import { google } from '@ai-sdk/google';
import { embed } from 'ai';
import dotenv from 'dotenv';

dotenv.config({ path: '.env.local' });

const qdrant = new QdrantClient({
  url: process.env.QDRANT_URL,
  apiKey: process.env.QDRANT_API_KEY,
});

const COLLECTION_NAME = 'textbook_v1';

// Ensure Embedding Dimension matches text-embedding-004 (768 dimensions)
const EMBEDDING_DIMENSION = 768;

async function main() {
  console.log('🚀 Starting Ingestion Pipeline (Gemini Edition)...');

  const files = await glob('docs/**/*.md', { ignore: 'node_modules/**' });
  console.log(`Found ${files.length} markdown files.`);

  // Warning for collection dimension mismatch if reusing OpenAI collection
  console.log(`NOTE: Ensure Qdrant collection is created with ${EMBEDDING_DIMENSION} dimensions.`);

  for (const file of files) {
    console.log(`Processing ${file}...`);
    const chunks = parseAndChunk(file);

    if (chunks.length === 0) continue;

    const points = [];
    
    for (const chunk of chunks) {
      // Generate Embedding (Gemini)
      const { embedding } = await embed({
        model: google.textEmbeddingModel('text-embedding-004'),
        value: chunk.content,
      });

      points.push({
        id: crypto.randomUUID(),
        vector: embedding,
        payload: {
          content: chunk.content,
          source_url: chunk.url,
          chapter_title: chunk.chapter_title,
          section_header: chunk.section_header,
        },
      });
    }

    await qdrant.upsert(COLLECTION_NAME, {
      points: points,
    });
    
    console.log(`  -> Indexed ${points.length} vectors.`);
  }

  console.log('✅ Ingestion Completel!');
}

main().catch(console.error);
