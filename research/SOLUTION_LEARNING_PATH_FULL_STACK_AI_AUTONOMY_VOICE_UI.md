# Full-Stack Learning Path — Unity UAS Mission + AI Autonomy + Voice + React Native UI

This document is a **guided learning + build order** across the research docs in this folder, starting from the Udacity Backyard Flyer mission and evolving into a more production-ready stack:

- deterministic control boundary (controller + arbiter + vehicle interface)
- telemetry + ops sidecar (MiNiFi C++)
- ROS2 Jazzy C++ runtime
- LeRobot learning pipeline (Shadow/Mixed/Policy)
- operator voice backend (HF ASR + Rasa + action server)
- Operator Gateway (REST + WS, network-facing)
- React Native (Expo) TypeScript frontend for monitoring + command/control

The organizing principle (repeated across the docs) is:

- **Actuation stays local and bounded** (controller + arbiter + vehicle node)
- **Everything else is best-effort** (telemetry shipping, voice/NLU, UI, data export)

---

## 0) How to use this learning path

There are two ways to follow this path:

- **Track A (Udacity-first):** implement the mission in Python first, then modularize, then move up-stack.
- **Track B (Systems-first):** start at ROS2/MiNiFi/LeRobot if you already know the mission and want the production architecture.

This guide is ordered so **Track A is the default**, but each step includes prerequisites so you can jump.

---

## 1) Reading/build order (recommended)

Each step lists:

- **Read**: which research doc(s) to open
- **Build**: what you should implement or stand up
- **Done when**: a simple validation you can check

### Step 1 — Mission baseline (Udacity requirement, Python)

Read:

- `README.md` (project requirement)
- `SOLUTION_MVP_PY.md` (MVP mental model)
- `SOLUTION_DESIGN_PY.md` (event-driven FSM + NED reminders)

Build:

- finish the TODOs in `backyard_flyer.py` (callbacks + transitions + waypoints)

Done when:

- Unity sim flies the **10m box at 3m altitude** and lands/disarms cleanly
- you can explain why 3m altitude is `down = -3.0` in NED telemetry, but UdaciDrone `cmd_position()` expects altitude-up

### Step 2 — Modularize the mission (Python, class-based)

Read:

- `SOLUTION_MODULAR_PY.md`
- `SOLUTION_DESIGN_FOLDER_STRUCTURE_PY.md`

Build:

- split planner/guards/FSM/adapters/logging into modules (while keeping the same mission behavior)

Done when:

- mission behavior is unchanged
- planner/guards can be unit-tested without simulator I/O

### Step 3 — “More real-time” framing (C++ architecture)

Read:

- `SOLUTION_DESIGN_CPP.md`
- `SOLUTION_DESIGN_FOLDER_STRUCTURE_CPP.md`

Build:

- no ROS2 yet: define the C++ architecture goals
  - fixed-rate loop (Model B)
  - explicit telemetry handoff
  - minimal allocations in hot path
  - `IVehicle` seam so you can test with `MockVehicle`

Done when:

- you can map each Python callback/state transition to a deterministic C++ equivalent

### Step 4 — Modular controller core (C++, unit-testable)

Read:

- `SOLUTION_MODULAR_CPP.md`

Build:

- implement the controller core library (planner/guards/FSM/controller) with a vehicle adapter interface

Done when:

- unit tests validate waypoint progression and state transitions without a simulator

### Step 5 — Telemetry-first contracts (JSONL)

Read:

- `SOLUTION_MINIFI_CPP.md` (telemetry pipeline + JSONL contract)

Build:

- standardize JSONL event logs (append-only):
  - `Logs/telemetry.jsonl`
  - `Logs/controller_status.jsonl`

Done when:

- you can tail logs and see:
  - state transitions
  - waypoint commands
  - periodic position samples (throttled)

### Step 6 — MiNiFi sidecar (edge shipping + routing)

Read:

- `SOLUTION_MINIFI_CPP.md`
- `SOLUTION_MINIFI_CPP_V2.md` (control-plane/ops boundary without entering actuation loop)

Build:

- MiNiFi flow to tail JSONL, validate/enrich, rate-limit, route, forward
- optional: ops hooks that call **control-plane** endpoints, not actuation

Done when:

- MiNiFi can be stopped/restarted without affecting flight control
- telemetry continues to be produced locally regardless of MiNiFi health

### Step 7 — ROS2 Jazzy runtime (C++ nodes + HTTP control-plane seam)

Read:

- `SOLUTION_ROS2_JAZZY_CPP.md`
- `SOLUTION_DESIGN_FOLDER_STRUCTURE_ROS2_MINIFI_CPP.md`

Build:

- ROS2 nodes: controller/vehicle/logger/arbiter
- `ros2_http_gateway_node` exposing localhost `/v1/*` endpoints

Done when:

- `POST /v1/mission/start` and `/v1/mission/stop` can safely start/stop a mission
- `controller_status.jsonl` reflects FSM state and loop health

### Step 8 — LeRobot enablement (learning-ready contracts)

Read:

- `SOLUTION_LEROBOT_ROS2_JAZZY_CPP.md` (Part 1: learning contracts + safe modes)
- `SOLUTION_LEROBOT_ROS2_JAZZY_CPP_PY_PART2.md` (Part 2: dataset/export/training/deploy/eval)
- `SOLUTION_DESIGN_FOLDER_STRUCTURE_LEROBOT_ROS2_MINIFI_CPP.md`

Build:

- add observation/action message contracts
- add `action_arbiter_node` gating and timeouts
- implement modes:
  - **Shadow**: policy runs, does not control
  - **Mixed**: policy controls only approved actions (initially `CMD_POSITION`)
  - **Policy**: policy is primary, still gated

Done when:

- you can collect episodes and run Shadow evaluation without affecting control
- switching modes is visible in:
  - `Logs/learning_events.jsonl`
  - `Logs/policy_eval.jsonl`

### Step 9 — Voice backend (HF ASR) as a control-plane client

Read:

- `SOLUTION_HF_VOICE_INTEG_LEROBOT_ROS2_MINIFI_CPP_PY.md`
- `SOLN_DES_DIR_STRUCTURE_HF_VOICE_INTEG_LEROBOT_ROS2_MINIFI_CPP_PY.md`

Build:

- voice service that:
  - captures audio
  - runs HF ASR
  - parses finite intents (MVP rules)
  - calls localhost `/v1/*`
  - appends audit to `Logs/operator_commands.jsonl`

Done when:

- voice can start/stop mission and switch modes (with confirmations handled in the client or backend)
- every attempt is observable in `operator_commands.jsonl`

### Step 10 — Rasa dialogue + confirmations + action server

Read:

- `SOLUTION_RASA_HF_VOICE_INTEG_LEROBOT_ROS2_MINIFI_CPP_PY_PART2.md`
- `SOLN_DES_DIR_STRUCTURE_RASA_HF_VOICE_INTEG_LEROBOT_ROS2_MINIFI_CPP_PY_PART2.md`

Build:

- Rasa NLU for intent classification
- dialogue rules for confirmations (especially Mixed/Policy)
- action server performs side effects (HTTP calls to `/v1/*`) and logs

Done when:

- “risky” intents require confirmation
- action server logs both intent + execution results to `operator_commands.jsonl`

### Step 11 — React Native UI (full-stack integration guide)

Read:

- `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK.md`
- `SOLN_DES_DIR_STRUCTURE_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK.md`

Build:

- Expo (TypeScript) app consuming:
  - REST snapshot `GET /api/v1/status`
  - WS stream `WS /api/v1/stream`

Done when:

- UI shows:
  - FSM stage progression
  - plots (altitude, speed, jitter)
  - recent operator commands

### Step 12 — UI rollout phases A → D (incremental capability)

Read:

- `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK_PHASEA.md`
- `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK_PHASEB.md`
- `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK_PHASEC.md`
- `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK_PHASED.md`
- `SOLN_DES_DIR_STRUCTURE_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK_PHASES_A_TO_D.md`

Build:

- Phase A: read-only dashboard
- Phase B: UI buttons for `/v1/*` (via gateway proxy), confirmations, audit
- Phase C: record/upload audio, HF ASR + Rasa backend returns transcript/intent/result
- Phase D: run/episode IDs + filtering, workflow per run

Done when:

- UI commands and voice commands both appear in the same audit stream (`operator_commands.jsonl`)
- run filtering prevents mixed-run event confusion

---

## 2) The “full stack” shape (mental model)

### Planes and boundaries

- **Actuation plane (hard boundary):**
  - controller + `action_arbiter_node` + `vehicle_interface_node`
  - must be bounded, deterministic, and local

- **Control plane (safe interface):**
  - localhost `/v1/*` endpoints
  - applies mode changes and mission start/stop at safe boundaries

- **Telemetry plane:**
  - JSONL logs written locally by logger node
  - MiNiFi tails and forwards; UI reads from gateway WS

- **Operator plane:**
  - voice service + Rasa action server + UI
  - never bypasses the arbiter/vehicle boundary

---

## 3) What you should implement first if you want a working UI quickly

If your goal is “get the phone dashboard working now”, the fastest practical sequence is:

1) Make sure `Logs/controller_status.jsonl` + `Logs/telemetry.jsonl` are being written.
2) Implement the Operator Gateway WS stream that emits parsed events from `Logs/*.jsonl`.
3) Follow **Phase A** UI guide.

Then add Phase B/C/D incrementally.

---

## 4) Quick reference: key logs and endpoints

### Logs (append-only)

- `Logs/telemetry.jsonl`
- `Logs/controller_status.jsonl`
- `Logs/learning_events.jsonl`
- `Logs/policy_eval.jsonl`
- `Logs/operator_commands.jsonl`

### Localhost control-plane (ROS2 gateway)

- `POST /v1/mission/start`
- `POST /v1/mission/stop`
- `POST /v1/policy/mode` (with `allow_action_types: [6]` initially)
- `POST /v1/episode/start`
- `POST /v1/episode/stop`
- `POST /v1/safety/stop`

### Network-facing Operator Gateway (for UI)

- `GET /api/v1/status`
- `WS /api/v1/stream`
- `POST /api/v1/cmd/...` (proxy to `/v1/*`)
- `POST /api/v1/voice/command` (optional)

---

## 5) Suggested milestones (so you don’t boil the ocean)

- **Milestone 1:** Python mission flies the box (Udacity done)
- **Milestone 2:** JSONL contract exists and is stable (telemetry groundwork)
- **Milestone 3:** ROS2 gateway can start/stop missions safely
- **Milestone 4:** Shadow mode evaluation works end-to-end
- **Milestone 5:** Voice commands are audited + confirmed
- **Milestone 6:** Phase B UI can operate missions with the same audit stream
