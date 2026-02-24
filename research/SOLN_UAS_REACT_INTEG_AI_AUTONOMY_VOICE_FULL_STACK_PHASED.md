# Phase D — Run/Episode IDs, Filtering, and “Workflow per Run” (React Native TypeScript Walkthrough)

This Phase D guide is the “code walkthrough” version of Phase D from:

- `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK.md`

Goal (Phase D):

- Tie the UI workflow view to specific `vehicle_id` and `run_id`.
- Generate and display a `run_id` in the UI.
- Filter streamed events by `(vehicle_id, run_id)`.
- Make it easy to start an episode, show current `episode_id`, and later find the exported artifacts.

Why Phase D matters:

- Without `run_id`, the UI becomes a “firehose viewer” and can mix events from multiple test runs.
- With `run_id`, the UI becomes a proper *mission replay / mission monitoring* tool.

---

## 0) Backend expectations

Your existing docs already standardize these fields in JSONL:

- `t_us`
- `vehicle_id`
- `run_id`

Phase D becomes simplest if the Operator Gateway supports either:

A) Server-side filtering:

- `WS /api/v1/stream?vehicle_id=sim-01&run_id=...`

or B) Client-side filtering:

- UI receives all events and filters locally.

This guide implements B (client-side) so it doesn’t require any gateway changes.

---

## 1) Add “active run” state to the UI store

### 1.1 Update `src/state/store.ts`

Add two fields and setters:

```ts
activeVehicleId: string;
activeRunId: string;
setActiveVehicleId: (v: string) => void;
setActiveRunId: (r: string) => void;
```

Then, inside `ingest(evt)`, drop events that don’t match:

```ts
const st = get();
if (evt.vehicle_id !== st.activeVehicleId) return;
if (evt.run_id !== st.activeRunId) return;
```

Initialize defaults:

- `activeVehicleId = "sim-01"`
- `activeRunId = "dev"` (or empty)

Important UX note:

- If you default to `dev`, but your backend run IDs are ISO timestamps, you’ll see no events. In practice, you want the UI to set the run_id it starts.

---

## 2) Generate a run_id at mission start

### 2.1 Update `CommandPanel` to use/store the run_id

In Phase B, `CommandPanel` used `new Date().toISOString()` in a local state.

In Phase D, move `run_id` into the global store and show it in the UI.

Example snippet:

```tsx
const runId = useAppStore((s) => s.activeRunId);
const setRunId = useAppStore((s) => s.setActiveRunId);

function newRunId() {
  return new Date().toISOString();
}

<Btn label="New Run ID" onPress={() => guarded(async () => setRunId(newRunId()))} />
<Btn label="Start Mission (Classical)" onPress={() => guarded(() => api.missionStart(runId, vehicleId))} />
```

Now the UI starts a mission with a run_id it knows, and filters events accordingly.

---

## 3) Add a Settings screen for selecting vehicle_id/run_id

### 3.1 `src/screens/SettingsScreen.tsx`

Use the simplest possible input fields.

```tsx
import React, { useState } from "react";
import { View, Text, TextInput, Pressable } from "react-native";
import { useAppStore } from "../state/store";

export function SettingsScreen() {
  const activeVehicleId = useAppStore((s) => s.activeVehicleId);
  const activeRunId = useAppStore((s) => s.activeRunId);
  const setActiveVehicleId = useAppStore((s) => s.setActiveVehicleId);
  const setActiveRunId = useAppStore((s) => s.setActiveRunId);

  const [v, setV] = useState(activeVehicleId);
  const [r, setR] = useState(activeRunId);

  return (
    <View style={{ padding: 12, gap: 12 }}>
      <Text style={{ fontWeight: "600" }}>Filtering</Text>

      <Text>Vehicle ID</Text>
      <TextInput value={v} onChangeText={setV} style={{ borderWidth: 1, padding: 10, borderRadius: 10 }} />

      <Text>Run ID</Text>
      <TextInput value={r} onChangeText={setR} style={{ borderWidth: 1, padding: 10, borderRadius: 10 }} />

      <Pressable
        onPress={() => {
          setActiveVehicleId(v.trim());
          setActiveRunId(r.trim());
        }}
        style={{ padding: 10, borderWidth: 1, borderRadius: 10, alignItems: "center" }}
      >
        <Text>Apply</Text>
      </Pressable>
    </View>
  );
}
```

Add this to tabs/navigation.

---

## 4) Episode ID handling

Your control-plane supports:

- `POST /v1/episode/start { "episode_id": "..." }`
- `POST /v1/episode/stop { "episode_id": "active" }`

Phase D recommendation:

- store `activeEpisodeId` in the UI store
- generate an episode id like `ep-<6hex>`

Then show it on the dashboard.

---

## 5) “Export run” links (minimal, non-invasive)

Your data layout doc already proposes:

- `data/raw_runs/<vehicle_id>/<run_id>/...`
- `data/lerobot/datasets/...`

The UI should not directly read filesystem paths from the robot host.

Minimal Phase D approach:

- Operator Gateway implements `GET /api/v1/runs/<vehicle_id>/<run_id>` returning URLs (or references) to artifacts.
- UI shows a list of links.

If you don’t want to implement that endpoint yet, Phase D can simply display the run_id and a copy button.

---

## 6) Phase D acceptance criteria

- UI can start a mission with a run_id that it generated.
- UI filters streamed events to only that run.
- UI can start/stop an episode and display the active episode_id.
- UI workflow view is stable and not mixing runs.
