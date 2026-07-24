# Planning documents

| File | What it is | Status |
|---|---|---|
| [firmware-implementation-outline.md](firmware-implementation-outline.md) | The original work-package breakdown (WP0–WP8) + milestones + open decisions, written before the firmware existed | WP0–WP6 + ml/ pipeline are **implemented** (see status note at the top of the file). D1 resolved: quad PCB drone. Remaining: WP7 simulator/tuning, flight testing, addons. |
| [avoidance-method-versions.md](avoidance-method-versions.md) | The parallel-versions strategy: 6 environment-perception versions (V1 ToF … V6 comms-only), 4 inter-robot localization versions (L1–L4), named builds A–F, and the shared `PerceptionSource`/`NeighborSource` contracts that make them swappable | Strategy in force. V1+V2 (builds A/A'/B) are implemented; V3–V6 and L2–L3 are specified-only. |

Current truth about what exists: [`../HANDBOOK.md`](../HANDBOOK.md).
What to do next: [`../ROADMAP.md`](../ROADMAP.md).
