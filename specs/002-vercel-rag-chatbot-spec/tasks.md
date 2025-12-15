# Task Breakdown: Vercel-Native RAG Chatbot (Phase 2)

**Status**: PENDING  
**Context**: [Phase 2 Plan](/specs/002-vercel-rag-chatbot-spec/plan.md) | [Phase 2 Spec](/specs/002-vercel-rag-chatbot-spec/spec.md)

---

## 1. Build-Time Ingestion (The Knowledge Base)

### TASK-2.1: Qdrant Setup & Configuration
*   **Description**: Initialize Qdrant Cloud client and create the collection.
*   **Inputs**: `QDRANT_URL`, `QDRANT_API_KEY`.
*   **Outputs**: `scripts/setup-qdrant.ts`.
*   **Acceptance Criteria**: Script runs successfully and creates a collection `textbook_v1` with cosine distance.

### TASK-2.2: Markdown Parser & Chunker
*   **Description**: Implement logic to parse Docusaurus Markdown files and split them into semantic chunks.
*   **Inputs**: `docs/**/*.md` files.
*   **Outputs**: `scripts/lib/chunker.ts`.
*   **Dependencies**: None.
*   **Acceptance Criteria**: 
    1.  Parses YAML frontmatter correctly.
    2.  Splits content by H2 headers and token limits (512 tokens).
    3.  Maintains metadata (URL, Title).

### TASK-2.3: Ingestion Pipeline Script
*   **Description**: The "main" script that orchestrates parsing, embedding (OpenAI), and upserting to Qdrant.
*   **Inputs**: `chunker.ts`, OpenAI API Key.
*   **Outputs**: `scripts/ingest.ts`.
*   **Dependencies**: TASK-2.1, TASK-2.2.
*   **Acceptance Criteria**: Running `npm run ingest` fully populates the Qdrant index with embeddings for all current docs.

---

## 2. Runtime API (The Brain)

### TASK-2.4: Vercel API Route Setup
*   **Description**: Configure parsing of Vercel Serverless Functions in Docusaurus.
*   **Inputs**: `docusaurus.config.ts` (middleware config if needed), `api/` directory.
*   **Outputs**: `api/chat/route.ts` (skeleton).
*   **Acceptance Criteria**: `curl POST /api/chat` returns 200 OK using Vercel local dev.

### TASK-2.5: Retrieval Logic Service
*   **Description**: Implement the `retrieve(query, mode, selection)` function.
*   **Inputs**: User query string.
*   **Outputs**: `src/lib/retrieval.ts`.
*   **Dependencies**: TASK-2.1.
*   **Acceptance Criteria**: 
    1.  Returns top-K chunks from Qdrant.
    2.  If `mode=selection`, returns constructed chunk from user selection.

### TASK-2.6: OpenAI RAG Orchestrator
*   **Description**: Implement the prompt construction and OpenAI stream call.
*   **Inputs**: Retrieved chunks, User query.
*   **Outputs**: `api/chat/route.ts` (full implementation).
*   **Dependencies**: TASK-2.5.
*   **Acceptance Criteria**:
    1.  System prompt enforces "Answer only from context".
    2.  Returns a streaming `Response` object compatible with `ai` SDK or raw streams.

---

## 3. Frontend Integration (The Interface)

### TASK-2.7: Chat Widget Component
*   **Description**: Create the React UI for the chat interface.
*   **Inputs**: Design requirements (mobile-first).
*   **Outputs**: `src/theme/ChatWidget/index.tsx`.
*   **Acceptance Criteria**:
    1.  Floating button in bottom-right.
    2.  Expandable chat window.
    3.  Input field + Message list.

### TASK-2.8: Selection Capture Logic
*   **Description**: Implement logic to detect text selection and prompt "Ask AI about this".
*   **Inputs**: `window.getSelection()`.
*   **Outputs**: `src/theme/Root.tsx` or global script.
*   **Acceptance Criteria**: Selecting text shows a tooltip/button that populates the chat input and sets `mode=selection`.

### TASK-2.9: Stream Handling & Markdown Rendering
*   **Description**: Parse the incoming SSE stream and render it as Markdown in the chat bubble.
*   **Inputs**: Stream from `/api/chat`.
*   **Outputs**: `src/theme/ChatWidget/MessageBubble.tsx`.
*   **Dependencies**: TASK-2.7.
*   **Acceptance Criteria**: Chat bubbles render Code blocks, Bold, and Links correctly.

---

## 4. Deployment & Validation

### TASK-2.10: Environment Configuration
*   **Description**: Set up `.env` and Vercel Project Config.
*   **Inputs**: API Keys.
*   **Outputs**: `.env.local`, Vercel Dashboard Settings.
*   **Acceptance Criteria**: Local dev works with real keys.

### TASK-2.11: E2E Validation (Hallucination Test)
*   **Description**: Manual verification task.
*   **Inputs**: Live Chatbot.
*   **Acceptance Criteria**:
    1.  "How do I bake a cake?" -> Refusal.
    2.  "What is a Node?" -> Correct answer cited from Module 1.
