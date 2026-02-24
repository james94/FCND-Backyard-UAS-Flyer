# Folder Structure Addendum — React UI Phases A–D

This document complements:

- `SOLN_DES_DIR_STRUCTURE_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK.md`
- `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK.md`

It shows how the repo can evolve across the UI rollout phases:

- Phase A: read-only dashboard
- Phase B: buttons (safe operator commands)
- Phase C: voice-from-UI (audio upload)
- Phase D: run/episode IDs + filtering

This is intentionally consistent with your existing operator/voice/Rasa placement and the ROS2 localhost gateway boundary.

---

## Phase A — UI read-only dashboard

What exists:

- Operator Gateway exposes:
  - `GET /api/v1/status`
  - `WS /api/v1/stream`
- Expo app renders:
  - Status + workflow + plots + event log

Tree (additions only):

```text
operator/
  gateway/
    app/
      server.py                # REST + WS
      tailer.py                 # tails Logs/*.jsonl
      types.py
    requirements.txt

  ui/
    mobile/
      src/
        api/
        state/
        components/             # StatusCard, MissionStepper, TelemetryCharts, TrajectoryPlot, EventLog
        screens/                # Dashboard, Telemetry
      App.tsx
```

---

## Phase B — Add UI command controls

What changes:

- Expo app adds:
  - `CommandPanel`
  - HTTP client for `POST /api/v1/cmd/*`

- Operator Gateway adds:
  - `/api/v1/cmd/*` routes that proxy to `ros2_http_gateway_node` `/v1/*`
  - gateway-side audit: append to `Logs/operator_commands.jsonl`

Tree delta:

```text
operator/
  gateway/
    app/
      server.py                # adds /api/v1/cmd/* handlers

  ui/
    mobile/
      src/
        api/
          http.ts              # POST helpers
        components/
          CommandPanel.tsx
        screens/
          DashboardScreen.tsx  # includes CommandPanel
```

---

## Phase C — Add voice-from-UI

What changes:

- Expo app adds recording and an upload call:
  - `expo-av`
  - `VoicePanel` component

- Operator Gateway adds:
  - `POST /api/v1/voice/command` endpoint
  - integration to HF ASR + Rasa + action server
  - appends result to `Logs/operator_commands.jsonl` and streams it

Tree delta:

```text
operator/
  gateway/
    app/
      server.py                # adds /api/v1/voice/command

  ui/
    mobile/
      src/
        components/
          VoicePanel.tsx
        screens/
          OperatorScreen.tsx   # optional
```

---

## Phase D — Run/episode IDs + filtering

What changes:

- Expo app adds:
  - `SettingsScreen` to set `vehicle_id` and `run_id`
  - filtering inside the store (drop events not matching)
  - run_id generation integrated with mission start

- Operator Gateway optionally adds:
  - server-side stream filtering (query params)
  - artifact discovery endpoints (optional)

Tree delta:

```text
operator/
  ui/
    mobile/
      src/
        screens/
          SettingsScreen.tsx
        state/
          store.ts             # activeVehicleId, activeRunId
```

---

## What stays constant across all phases

- ROS2 actuation boundary: `CommandIntent` remains authoritative behind `action_arbiter_node`.
- UI never depends on ROS2/DDS.
- UI consumes structured events over WS (not raw JSONL).
- `Logs/operator_commands.jsonl` remains the unified audit stream for button + voice commands.
