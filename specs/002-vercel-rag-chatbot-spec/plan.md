# Implementation Plan: Phase 2 - Vercel-Native RAG Chatbot

**Branch**: `specs/002-vercel-rag-chatbot-spec` | **Date**: 2025-12-15 | **Context**: [Phase 1 Plan](/specs/001-ai-robotics-textbook-spec/plan.md)
**Objective**: Deploy a serverless, cost-free RAG chatbot on Vercel to allow readers to query the textbook content.

## Summary

Implement a "Chat with the Book" feature using a Vercel-native architecture. The system will ingest textbook content into a Qdrant vector database at build time and provide a streaming chat interface via Vercel Edge Functions at runtime. It relies on OpenAI for reasoning and Neon/Qdrant for persistence, strictly adhering to the "Vercel Free Tier" constraint.

## Technical Context

**Infrastructure**: Vercel (Edge Functions, static hosting), Neon (Serverless Postgres), Qdrant Cloud (Vector DB).
**AI Stack**: OpenAI API (GPT-4o-mini/GPT-3.5-turbo for cost), OpenAI Embeddings (text-embedding-3-small).
**Frameworks**: Next.js (if moving to Next) OR Docusaurus swizzled React components + Vercel Serverless Functions. *Decision: Keep Docusaurus, add Vercel API routes.*
**Languages**: TypeScript, Python (Ingestion scripts).

## Constitution Check

*   [x] **Hosting**: Vercel Free Tier only.
*   [x] **Architecture**: Stateless, Route Handlers.
*   [x] **Retrieval**: Strict RAG (Full-book & Selected-text modes).
*   [x] **Security**: Secrets in Env Vars, no client-side keys.

## Execution Roadmap & Phases

### Phase 2.1: Build-Time Ingestion (The Knowledge Base)
*   **Goal**: Transform Markdown content into vector embeddings.
*   **Input**: `docs/` Markdown files.
*   **Process**:
    1.  **Parsing**: Extract front matter (title, ID) and clean content.
    2.  **Chunking**: Semantic chunking (headers, paragraphs) with overlap.
    3.  **Embedding**: Generate vectors using `text-embedding-3-small`.
    4.  **Indexing**: Upsert to Qdrant Cloud with metadata (chapter_id, page_url).
*   **Artifact**: `scripts/ingest.ts` (runnable via GitHub Actions or locally).

### Phase 2.2: Runtime Architecture (The Brain)
*   **Goal**: Handle chat queries statelessly.
*   **Endpoints**:
    *   `api/chat`: Main RAG endpoint.
*   **Flow**:
    1.  **Auth**: Validate Origin/CORS.
    2.  **Embedding**: Embed user query.
    3.  **Retrieval**: Query Qdrant for top-k similar chunks (filter by chapter if needed).
    4.  **Generation**: Construct prompt with retrieved context + system instructions.
    5.  **Response**: Stream token-by-token response to client.

### Phase 2.3: Frontend Integration (The Interface)
*   **Goal**: Embed chat cleanly into Docusaurus.
*   **Components**:
    *   `ChatWidget`: Floating bubble or sidebar panel.
    *   `ContextSelector`: Toggle between "Full Book" and "This Page".
*   **Interaction**: capture text selection events to pre-fill "Ask about this..."

### Phase 2.4: Deployment & Validation
*   **Goal**: Live on Vercel.
*   **Config**: `vercel.json` configuration for rewrites/functions.
*   **Env Vars**: `OPENAI_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`.
*   **Testing**:
    *   **Hallucination Test**: Ask questions about non-existent content (must refuse).
    *   **Grounding Test**: Verify answers cite specific chapters.

## Quality Gates

1.  **Cost Safety**: Does the ingestion script have a fail-safe to prevent massive API usage?
2.  **Latency**: Is the time-to-first-token < 1.5s?
3.  **Mobile**: Does the chat widget obstruct reading on phones?

## Data Schema (Qdrant)

```json
{
  "payload": {
    "content": "Text chunk...",
    "source_url": "/docs/module-01/foundations",
    "chapter_title": "ROS 2 Nervous System",
    "section_header": "Nodes & Topics"
  },
  "vector": [...]
}
```
