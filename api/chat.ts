import { createGoogleGenerativeAI } from '@ai-sdk/google';
import { streamText } from 'ai';
import { retrieveContext } from '../src/lib/qdrant'; // Adjusted path
import { ChatRequest } from '../src/lib/rag-types'; // Adjusted path

export const config = {
  runtime: 'edge',
};

// Create Google Provider explicitly to force key loading
const google = createGoogleGenerativeAI({
  baseURL: 'https://generativelanguage.googleapis.com/v1beta',
  apiKey: process.env.GOOGLE_GENERATIVE_AI_API_KEY,
});

export default async function handler(req: Request) {
  console.log("DEBUG: Env Key available?", !!process.env.GOOGLE_GENERATIVE_AI_API_KEY);
  
  if (req.method !== 'POST') {
    return new Response('Method Not Allowed', { status: 405 });
  }

  try {
    const body = await req.json() as ChatRequest;
    const { messages, mode, context } = body;
    const lastMessage = messages[messages.length - 1];

    // 1. Retrieve Context
    const contextChunks = await retrieveContext(
      lastMessage.content,
      mode,
      context?.selection
    );

    // 2. Construct System Prompt
    const contextBlock = contextChunks
      .map((c) => `[Source: ${c.metadata.chapter_title}]\n${c.content}`)
      .join('\n\n');

    const systemPrompt = `You are the AI Teaching Assistant for "Physical AI & Humanoid Robotics".
    
RULES:
1. Answer ONLY using the provided Context.
2. If the answer is not in the Context, say "I cannot find this information in the textbook."
3. Be concise, professional, and educational.

CONTEXT:
${contextBlock}
`;

    // 3. Call Gemini (Streaming)
    // Convert to CoreMessage format explicitly to satisfy TypeScript
    const coreMessages = messages.map(m => ({
      role: m.role as 'user' | 'assistant' | 'system',
      content: m.content,
    }));

    const result = await streamText({
      model: google('gemini-1.5-flash'),
      system: systemPrompt,
      messages: coreMessages,
    });

    // Use toTextStreamResponse as suggested by the error (compatible with ai sdk v5 text streams)
    return result.toTextStreamResponse();

  } catch (error) {
    console.error('Chat API Error:', error);
    return new Response(JSON.stringify({ error: 'Internal Server Error' }), {
      status: 500,
      headers: { 'Content-Type': 'application/json' },
    });
  }
}
