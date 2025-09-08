Pete Knightykell — Forebrain

Purpose
- Headless laptop/PC with a decent GPU for LLMs, vision models, and task planning.
- Translates natural‑language goals into structured actions; coordinates with cerebellum.

Notes
- Interfaces: gRPC/ROS 2 bridge/REST/WebSocket to cerebellum; optional camera ingest.
- Manages model serving (LLM/VLM), retrieval, and tool orchestration.

Suggested layout (to be added later)
- services/: model servers, planners, perception pipelines
- adapters/: ROS 2 bridge, protocol gateways
- config/: model weights/paths, runtime settings
- docker/: container builds and compose files
- docs/: ops runbooks and performance notes

Getting started
- Define intent schema and action protocol shared with cerebellum.
- Stand up a small LLM first; stub planner → simulated tasks.
