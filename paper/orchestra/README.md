# PaperOrchestra run (Claude backend)

Inputs (`idea.md`, `experimental_log.md`, `conference_guidelines.md`, `citation_map.json`, `template.tex`), the
outline, the AgentReview reviews and the refinement log of running the PaperOrchestra stages with Claude
sub-agents instead of the Gemini/OpenAI backend. `refinement_log.md` records the scores and decisions.

`prompt_*.txt` are copied from google-research/paper-orchestra (`methods/prompts/`, `autoraters/agent_review.py`,
`utils/prompt_utils.py`), Copyright 2026 Google LLC, licensed under the Apache License, Version 2.0
(https://www.apache.org/licenses/LICENSE-2.0). Only the `{cutoff_date}` placeholder and brace escaping were
changed; the reviewer prompt is assembled from the AgentReview persona and instruction strings.
