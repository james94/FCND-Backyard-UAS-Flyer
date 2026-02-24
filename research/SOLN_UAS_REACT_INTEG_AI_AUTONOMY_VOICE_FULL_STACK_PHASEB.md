# Phase B — Add UI Buttons for Safe `/v1/*` Operator Commands (TypeScript Walkthrough)

This Phase B guide is the “code walkthrough” version of Phase B from:

- `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK.md`

Goal (Phase B):

- Keep Phase A read-only streaming dashboard.
- Add **UI buttons** that issue the same operator commands you already support via CLI/voice.
- Add **confirmations** for risky actions (Mixed/Policy mode).
- Ensure every UI command attempt is logged to `Logs/operator_commands.jsonl` (gateway-side audit).

Safety boundary (unchanged):

- UI does not publish actuation.
- UI calls only the Operator Gateway command endpoints (which proxy to the ROS2 localhost gateway).

---

## 0) Backend contracts used in Phase B

Operator Gateway endpoints (recommended):

- `POST /api/v1/cmd/mission/start` → proxies `/v1/mission/start`
- `POST /api/v1/cmd/mission/stop` → proxies `/v1/mission/stop`
- `POST /api/v1/cmd/policy/mode` → proxies `/v1/policy/mode`
- `POST /api/v1/cmd/episode/start` → proxies `/v1/episode/start`
- `POST /api/v1/cmd/episode/stop` → proxies `/v1/episode/stop`
- `POST /api/v1/cmd/safety/stop` → proxies `/v1/safety/stop`

Note: `CMD_POSITION=6` matches your ROS2 `CommandIntent.msg` enum.

---

## 1) UI code delta from Phase A

Phase B adds:

- an HTTP client that can `POST` JSON with timeouts
- a `CommandPanel` component
- confirmations for Mixed/Policy

---

## 2) Implement bounded HTTP helpers

### 2.1 `src/api/http.ts` (replace Phase A version)

```ts
import { API_BASE_URL } from "./config";

async function postJson<TReq extends object, TRes>(path: string, body: TReq): Promise<TRes> {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 1200);
  try {
    const res = await fetch(API_BASE_URL + path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
      signal: controller.signal,
    });
    const text = await res.text();
    if (!res.ok) throw new Error(`HTTP ${res.status}: ${text}`);
    return (text ? (JSON.parse(text) as TRes) : ({} as TRes));
  } finally {
    clearTimeout(timeout);
  }
}

async function getJson<TRes>(path: string): Promise<TRes> {
  const res = await fetch(API_BASE_URL + path);
  if (!res.ok) throw new Error(`HTTP ${res.status}`);
  return (await res.json()) as TRes;
}

export const api = {
  status: () => getJson<unknown>("/api/v1/status"),

  missionStart: (run_id: string, vehicle_id: string) =>
    postJson("/api/v1/cmd/mission/start", { run_id, vehicle_id, mode: "classical" }),

  missionStop: (reason = "operator_request") => postJson("/api/v1/cmd/mission/stop", { reason }),

  policyMode: (mode: "shadow" | "mixed" | "policy") =>
    postJson("/api/v1/cmd/policy/mode", { mode, allow_action_types: [6] }),

  episodeStart: (episode_id: string) => postJson("/api/v1/cmd/episode/start", { episode_id }),
  episodeStop: () => postJson("/api/v1/cmd/episode/stop", { episode_id: "active" }),

  safetyStop: (behavior: "stop" | "land") => postJson("/api/v1/cmd/safety/stop", { behavior }),
};
```

---

## 3) Add a Command Panel with confirmations

### 3.1 `src/components/CommandPanel.tsx`

Use `Alert` to keep the UX minimal.

```tsx
import React, { useState } from "react";
import { View, Text, Pressable, Alert } from "react-native";
import { api } from "../api/http";

function Btn({ label, onPress, disabled }: { label: string; onPress: () => Promise<void>; disabled?: boolean }) {
  return (
    <Pressable
      disabled={disabled}
      onPress={() => void onPress()}
      style={{
        padding: 10,
        borderWidth: 1,
        borderRadius: 10,
        alignItems: "center",
        opacity: disabled ? 0.5 : 1,
      }}
    >
      <Text>{label}</Text>
    </Pressable>
  );
}

export function CommandPanel() {
  const [busy, setBusy] = useState(false);
  const [vehicleId] = useState("sim-01");
  const [runId] = useState(() => new Date().toISOString());

  async function guarded(fn: () => Promise<void>) {
    if (busy) return;
    setBusy(true);
    try {
      await fn();
    } catch (e: any) {
      Alert.alert("Command failed", String(e?.message ?? e));
    } finally {
      setBusy(false);
    }
  }

  function confirm(label: string, yes: () => void) {
    Alert.alert("Confirm", label, [
      { text: "Cancel", style: "cancel" },
      { text: "Yes", style: "destructive", onPress: yes },
    ]);
  }

  return (
    <View style={{ gap: 10, padding: 12, borderWidth: 1, borderRadius: 10 }}>
      <Text style={{ fontWeight: "600" }}>Commands</Text>

      <Btn disabled={busy} label="Start Mission (Classical)" onPress={() => guarded(() => api.missionStart(runId, vehicleId).then(() => Promise.resolve()))} />
      <Btn disabled={busy} label="Stop Mission" onPress={() => guarded(() => api.missionStop())} />

      <Btn disabled={busy} label="Policy Mode: Shadow" onPress={() => guarded(() => api.policyMode("shadow"))} />

      <Btn
        disabled={busy}
        label="Policy Mode: Mixed (CMD_POSITION)"
        onPress={() =>
          guarded(
            () =>
              new Promise<void>((resolve) =>
                confirm("Enable MIXED mode (CMD_POSITION only)?", () => {
                  void api.policyMode("mixed").then(() => resolve());
                }),
              ),
          )
        }
      />

      <Btn
        disabled={busy}
        label="Policy Mode: Policy (CMD_POSITION)"
        onPress={() =>
          guarded(
            () =>
              new Promise<void>((resolve) =>
                confirm("Enable POLICY mode (CMD_POSITION only)?", () => {
                  void api.policyMode("policy").then(() => resolve());
                }),
              ),
          )
        }
      />

      <Btn disabled={busy} label="Episode Start" onPress={() => guarded(() => api.episodeStart(`ep-${Math.random().toString(16).slice(2, 8)}`))} />
      <Btn disabled={busy} label="Episode Stop" onPress={() => guarded(() => api.episodeStop())} />

      <Btn disabled={busy} label="EMERGENCY: Land" onPress={() => guarded(() => api.safetyStop("land"))} />
    </View>
  );
}
```

---

## 4) Place the panel on the Dashboard

In `DashboardScreen.tsx`, add:

```tsx
import { CommandPanel } from "../components/CommandPanel";
```

and render it (example):

```tsx
<CommandPanel />
```

---

## 5) Make sure UI commands show up in the UI (audit loop)

Phase B success depends on end-to-end visibility:

- UI sends `POST /api/v1/cmd/...`
- Operator Gateway appends an `operator_command` record to `Logs/operator_commands.jsonl`
- Operator Gateway also emits an `operator_commands` stream event over WS
- UI sees the event in `EventLog` (and optionally in a dedicated “Operator” screen)

If you don’t see your UI button presses show up in the event log, the missing piece is gateway-side auditing.

---

## 6) Phase B acceptance criteria

- Button commands are equivalent to CLI/voice commands.
- Mixed/Policy actions require explicit confirmation.
- Every button press appears in `operator_commands` stream (and in `operator_commands.jsonl`).
- UI remains stable if backend is down (clear errors, no crashes).
