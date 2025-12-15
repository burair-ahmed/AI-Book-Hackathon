# Constitution: Physical AI & Humanoid Robotics

**Title:** "Physical AI & Humanoid Robotics: Embodied Intelligence in the Real World"  
**System:** Spec-Kit Plus  
**Implementation:** Claude Code  
**Deployment:** Docusaurus on GitHub Pages + Vercel (Chatbot)

---

## Phase 1: The Textbook (Core)

### 1. Educational Philosophy
*   **Bridge Software & Embodiment:** The book must bridge AI software systems with physical robotic embodiment.
*   **Core Emphasis:** Emphasize embodied intelligence, real-world physics, and human-centered design.
*   **Target Audience:** Assume readers already understand basic AI/ML concepts.

### 2. Technical Standards
*   **Middleware:** ROS 2 (Humble/Iron) is the canonical robotics middleware.
*   **Platforms:** NVIDIA Isaac Sim, Isaac ROS, Gazebo, Unity, and Jetson are first-class platforms.
*   **OS:** All examples must be Linux-first (Ubuntu 22.04).

### 3. Content Requirements
*   **Module Structure:** Every module must include:
    *   Conceptual theory
    *   Architecture diagrams (described textually)
    *   Code snippets (Python / ROS 2)
    *   Simulation workflows
    *   Real-world deployment notes
*   **Capstone:** Capstone must integrate Vision-Language-Action (VLA).

### 4. Documentation & UX
*   **Format:** Written in Markdown compatible with Docusaurus.
*   **Tone:** Modern, clean, professional tone. No fluff, no marketing language.
*   **Responsiveness:** Mobile-perfect responsive reading experience.

### 5. Output Discipline
*   **Completeness:** No skipped steps.
*   **Hardware:** No assumed hardware access.
*   **Alternatives:** Explicit hardware alternatives (local vs cloud).

---

## Phase 2: The Chatbot (Vercel-Native RAG)

### 6. Hosting & Cost
*   **Infrastructure:** MUST run entirely on Vercel Free Tier.
*   **Servers:** No paid servers or self-hosted backends allowed.
*   **Compute:** All logic MUST run as Vercel Serverless or Edge Functions.

### 7. Architecture
*   **Backend:** Logic implemented via Vercel Route Handlers (`/api`).
*   **State:** Stateless execution model (no in-memory sessions).
*   **Persistence:** External persistence ONLY via Neon Postgres (SQL) and Qdrant Cloud (Vectors).

### 8. Retrieval Discipline
*   **Methodology:** Strict Retrieval-Augmented Generation (RAG).
*   **Modes:**
    1.  **Full-Book Retrieval:** Search across the entire textbook.
    2.  **Selected-Text Retrieval:** Hard isolation to user's current selection.

### 9. LLM Constraints
*   **SDKs:** Use Vercel AI SDK (`@ai-sdk/google`).
*   **Models:** Google Gemini 1.5 Flash (Free Tier).
*   **Browsing:** No internet browsing allowed.
*   **Grounding:** Answers MUST be grounded in retrieved context only.

### 10. Security
*   **Secrets:** API keys stored in Vercel Environment Variables.
*   **Client:** No secrets ever exposed in client-side code.
*   **CORS:** API access locked to the book domain.

### 11. UX
*   **Integration:** Embedded directly inside the Docusaurus interface.
*   **Responsive:** Mobile-first design.
*   **Context:** Chatbot must be aware of the current chapter and page.
