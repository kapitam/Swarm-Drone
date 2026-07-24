# Documentation map

Reading order depends on what you came for:

- **"I want to understand/continue this project"** →
  [HANDBOOK.md](HANDBOOK.md) — the self-contained handoff: project state,
  architecture, fork matrix awaiting decision, safety + bring-up procedures,
  honest list of what is untested, conventions, cheat sheet.
- **"What should be built next?"** → [ROADMAP.md](ROADMAP.md) — ordered next
  steps, each with the research doc/section that already answered the design
  questions, the code seam to modify, and acceptance criteria. Written so a
  weaker agent can pick up any single item without re-research.
- **"Why was X chosen?"** → [research/](research/README.md) — annotated index
  of the 10 research documents (swarm algorithms, obstacle avoidance, ESP32
  feasibility, RTOS, ML depth verdict, camera version spec, V1 ToF recipe,
  V2 ML pipeline).
- **"What was the plan?"** → [plan/](plan/README.md) — work packages and the
  parallel versions/builds strategy, with implementation status notes.

Code-side documentation lives next to the code:
[`../lib/swarmcore/README.md`](../lib/swarmcore/README.md) (core algorithms),
[`../src/README.md`](../src/README.md) (firmware layer),
[`../ml/README.md`](../ml/README.md) (training pipeline),
[`../test/README.md`](../test/README.md) (unit tests).
