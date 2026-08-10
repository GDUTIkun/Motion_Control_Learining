## Knowledge and code tool routing

### Graphify — research knowledge layer (merged graph)

- papers, literature, notes, research concepts, and their relationships
- locating the documented implementation entry point for a concept
- cross-document knowledge discovery

Graphify graph (absolute path):
  D:\Workspace\graphify-out\merged-graph.json

How to query it:
  graphify query "..." --graph D:\Workspace\graphify-out\merged-graph.json
  graphify explain "<node label></node>" --graph D:\Workspace\graphify-out\merged-graph.json

Rules:

- Only query the existing merged graph. NEVER run `graphify extract` to build
  graphify-out/ in the current directory — it is not there and will not help.
- If CWD has no graphify-out/graph.json, do not invoke the graphify skill for
  code questions.
- Do not use Graphify for live code structure.

### codebase-memory-mcp — live code structure layer

Use for MATLAB, Python, C/C++ source structure, functions, classes, modules,
call relationships, implementation tracing, change-impact analysis.

Rules:

- Requires the repo indexed first (index_repository). The graph is only as
  fresh as the last index — re-index after big refactors.
- Treat this as authoritative for code structure.

### Connecting research to implementation

1. Query Graphify (merged graph) for the concept + documented location.
2. Query codebase-memory-mcp to verify current files, symbols, call paths.
3. If they disagree, codebase-memory-mcp wins (live code beats stale docs).



可以跟据全局AGENTS.md调用claude帮助干活
