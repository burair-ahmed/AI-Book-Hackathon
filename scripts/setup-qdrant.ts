import { QdrantClient } from '@qdrant/js-client-rest';
import dotenv from 'dotenv';
dotenv.config({ path: '.env.local' });

const qdrant = new QdrantClient({
  url: process.env.QDRANT_URL,
  apiKey: process.env.QDRANT_API_KEY,
});

const COLLECTION_NAME = 'textbook_v1';
const EMBEDDING_DIMENSION = 768; // Gemini text-embedding-004

async function main() {
  console.log(`Checking Qdrant connection to ${process.env.QDRANT_URL}...`);
  
  const collections = await qdrant.getCollections();
  const exists = collections.collections.some(c => c.name === COLLECTION_NAME);

  if (exists) {
    console.log(`Collection '${COLLECTION_NAME}' already exists.`);
    // WARNING: If user had OpenAI collection (1536 dim), strict error will occur.
    // For safety in this migration, we wont auto-delete, but warn.
    return;
  }

  console.log(`Creating collection '${COLLECTION_NAME}' (Dim: ${EMBEDDING_DIMENSION})...`);
  await qdrant.createCollection(COLLECTION_NAME, {
    vectors: {
      size: EMBEDDING_DIMENSION, 
      distance: 'Cosine',
    },
  });

  console.log('Collection created successfully!');
}

main().catch(console.error);
