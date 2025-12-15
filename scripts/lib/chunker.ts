import fs from 'fs';
import path from 'path';
import matter from 'gray-matter';

export interface Chunk {
  id: string;
  url: string;
  chapter_title: string;
  section_header: string;
  content: string;
}

// Helper to clean markdown syntax if needed
function cleanText(text: string): string {
  return text.replace(/\s+/g, ' ').trim();
}

export function parseAndChunk(filePath: string, baseUrl: string = '/docs'): Chunk[] {
  const fileContent = fs.readFileSync(filePath, 'utf-8');
  const { data, content } = matter(fileContent);
  const chunks: Chunk[] = [];

  // Metadata
  const id = data.id || path.basename(filePath, '.md');
  const title = data.title || id;
  const slug = filePath.split('docs')[1].replace(/\\/g, '/').replace('.md', '');
  const url = `${baseUrl}${slug}`;

  // Simple splitting by H2 keys (## )
  // strict splitting to keep context with headers
  const sections = content.split(/^##\s+/m);

  sections.forEach((section, index) => {
    if (!section.trim()) return;

    // Use specific logic to extract header vs body
    const lines = section.trim().split('\n');
    const header = index === 0 ? 'Introduction' : lines[0];
    const body = index === 0 ? lines.join('\n') : lines.slice(1).join('\n');

    // Skip empty sections
    if (!body.trim()) return;

    // Basic Token Limit Splitting (approx 500 chars ~ 100-150 tokens)
    // For a real prod system, use a tokenizer. For this hackathon, char count is safe.
    const MAX_CHUNK_SIZE = 1500; // chars
    let currentBody = body;

    let chunkIndex = 0;
    while (currentBody.length > 0) {
      const slice = currentBody.slice(0, MAX_CHUNK_SIZE);
      
      chunks.push({
        id: `${id}-${header}-${chunkIndex}`,
        url,
        chapter_title: title,
        section_header: header,
        content: `[Chapter: ${title} | Section: ${header}]\n${slice}`, // Prompt engineering embedded in chunk
      });

      currentBody = currentBody.slice(MAX_CHUNK_SIZE);
      chunkIndex++;
    }
  });

  return chunks;
}
