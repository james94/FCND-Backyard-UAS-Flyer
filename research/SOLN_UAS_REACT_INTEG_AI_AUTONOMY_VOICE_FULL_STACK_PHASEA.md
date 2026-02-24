# Phase A — React Native UI as Read-Only Dashboard (CLI/Voice still starts missions)

This Phase A guide is the “code walkthrough” version of Phase A from:

- `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK.md`

Goal (Phase A):

- Keep using CLI/voice (HF ASR + Rasa) to issue commands.
- Build a React Native (Expo) UI that is **read-only**:
  - shows mission FSM workflow (Manual → Arming → Takeoff → Waypoint → Landing → Disarming)
  - shows real-time status + telemetry plots
  - shows recent operator command events (from `operator_commands.jsonl`)

Safety boundary (unchanged):

- UI is not allowed to execute actuation.
- UI should not call `/v1/*` command endpoints in Phase A.

---

## 0) Backend assumptions (grounded in your existing docs)

Signals and contracts already defined by your backend docs:

- Control-plane exists on the ROS2 machine as localhost `/v1/*` (used by voice/MiNiFi).
- Telemetry/status is written under `Logs/*.jsonl` and tailed by MiNiFi.

For the UI, you should not stream raw JSONL files. Instead, Phase A assumes a **network-facing Operator Gateway** exists that provides:

- `GET /api/v1/status` (snapshot)
- `WS /api/v1/stream` (structured event stream)

Event envelope (recommended):

```json
{
  "stream": "telemetry|controller_status|learning_events|policy_eval|operator_commands",
  "t_us": 1700000001123456,
  "vehicle_id": "sim-01",
  "run_id": "2026-02-24T01-23-45Z",
  "event_type": "position_sample",
  "payload": {"position_ned": {"north": 1.2, "east": 0.3, "down": -3.0}}
}
```

---

## 1) Expo app setup (TypeScript)

Create the app (example):

```bash
npx create-expo-app operator-ui --template
cd operator-ui
```

Install dependencies used in Phase A (read-only):

```bash
npm i @react-navigation/native @react-navigation/bottom-tabs
npx expo install react-native-screens react-native-safe-area-context

npm i victory-native
npx expo install react-native-svg

npm i zustand
```

---

## 2) Phase A file structure

Inside the Expo app:

```text
operator-ui/
  src/
    api/
      config.ts
      types.ts
      ws.ts
      http.ts
    state/
      store.ts
      selectors.ts
    components/
      StatusCard.tsx
      MissionStepper.tsx
      TelemetryCharts.tsx
      TrajectoryPlot.tsx
      EventLog.tsx
    screens/
      DashboardScreen.tsx
      TelemetryScreen.tsx
      SettingsScreen.tsx
  App.tsx
```

---

## 3) Implement the API config

### 3.1 `src/api/config.ts`

```ts
export const API_BASE_URL =
  process.env.EXPO_PUBLIC_API_BASE_URL?.replace(/\/$/, "") ?? "http://127.0.0.1:8088";

export const WS_URL = API_BASE_URL.replace(/^http/, "ws") + "/api/v1/stream";
```

Dev note:

- On phones, `127.0.0.1` is the phone. Set `EXPO_PUBLIC_API_BASE_URL` to the gateway host’s LAN IP.

---

## 4) Define UI types

### 4.1 `src/api/types.ts`

```ts
export type StreamName =
  | "telemetry"
  | "controller_status"
  | "learning_events"
  | "policy_eval"
  | "operator_commands";

export type Ned = { north: number; east: number; down: number };

export type UiEventEnvelope = {
  stream: StreamName;
  t_us: number;
  vehicle_id: string;
  run_id: string;
  event_type: string;
  payload: Record<string, unknown>;
};

export type ControllerStatus = {
  controller_state: string;
  in_mission: boolean;
  loop_rate_hz: number;
  loop_jitter_ms_p95: number;
  t_us: number;
  vehicle_id: string;
  run_id: string;
};

export type StatusSnapshot = {
  controller?: ControllerStatus;
  policy_mode?: "CLASSICAL" | "SHADOW" | "MIXED" | "POLICY";
};
```

---

## 5) WebSocket stream connector

### 5.1 `src/api/ws.ts`

```ts
import { WS_URL } from "./config";
import type { UiEventEnvelope } from "./types";

export function connectStream(
  onEvent: (evt: UiEventEnvelope) => void,
  onStatus?: (s: "connected" | "disconnected" | "error") => void,
): () => void {
  const ws = new WebSocket(WS_URL);
  ws.onopen = () => onStatus?.("connected");
  ws.onclose = () => onStatus?.("disconnected");
  ws.onerror = () => onStatus?.("error");
  ws.onmessage = (msg) => {
    try {
      onEvent(JSON.parse(String(msg.data)) as UiEventEnvelope);
    } catch {
      // ignore
    }
  };
  return () => ws.close();
}
```

---

## 6) Minimal HTTP snapshot (optional)

### 6.1 `src/api/http.ts`

```ts
import { API_BASE_URL } from "./config";
import type { StatusSnapshot } from "./types";

export async function getStatus(): Promise<StatusSnapshot> {
  const res = await fetch(API_BASE_URL + "/api/v1/status");
  if (!res.ok) throw new Error(`HTTP ${res.status}`);
  return (await res.json()) as StatusSnapshot;
}
```

---

## 7) State store (read-only)

### 7.1 `src/state/store.ts`

```ts
import { create } from "zustand";
import type { ControllerStatus, Ned, StatusSnapshot, UiEventEnvelope } from "../api/types";

type Sample = { t_us: number; pos?: Ned; vel?: Ned };

type Store = {
  snapshot: StatusSnapshot;
  controller?: ControllerStatus;
  samples: Sample[];
  events: UiEventEnvelope[];
  wsStatus: "connected" | "disconnected" | "error";

  setSnapshot: (s: StatusSnapshot) => void;
  setWsStatus: (s: Store["wsStatus"]) => void;
  ingest: (evt: UiEventEnvelope) => void;
};

const MAX_SAMPLES = 600;
const MAX_EVENTS = 250;

export const useAppStore = create<Store>((set, get) => ({
  snapshot: {},
  samples: [],
  events: [],
  wsStatus: "disconnected",

  setSnapshot: (snapshot) => set({ snapshot }),
  setWsStatus: (wsStatus) => set({ wsStatus }),

  ingest: (evt) => {
    const st = get();
    const events = [evt, ...st.events].slice(0, MAX_EVENTS);

    if (evt.stream === "controller_status") {
      const p: any = evt.payload;
      const controller: ControllerStatus = {
        controller_state: String(p.controller_state ?? ""),
        in_mission: Boolean(p.in_mission),
        loop_rate_hz: Number(p.loop_rate_hz ?? 0),
        loop_jitter_ms_p95: Number(p.loop_jitter_ms_p95 ?? 0),
        t_us: Number(p.t_us ?? evt.t_us),
        vehicle_id: String(p.vehicle_id ?? evt.vehicle_id),
        run_id: String(p.run_id ?? evt.run_id),
      };
      set({ events, controller });
      return;
    }

    if (evt.stream === "telemetry" && evt.event_type === "position_sample") {
      const p: any = evt.payload;
      const samples =
        [{ t_us: evt.t_us, pos: p.position_ned as Ned | undefined, vel: p.velocity_ned as Ned | undefined }, ...st.samples]
          .slice(0, MAX_SAMPLES);
      set({ events, samples });
      return;
    }

    set({ events });
  },
}));
```

### 7.2 `src/state/selectors.ts` (FSM stage mapping)

```ts
const ORDER = ["MANUAL", "ARMING", "TAKEOFF", "WAYPOINT", "LANDING", "DISARMING"] as const;

export function normalizeControllerState(s?: string): (typeof ORDER)[number] | "UNKNOWN" {
  if (!s) return "UNKNOWN";
  const x = s.trim().toUpperCase();
  if (x.includes("ARM")) return "ARMING";
  if (x.includes("TAKEOFF")) return "TAKEOFF";
  if (x.includes("WAYPOINT")) return "WAYPOINT";
  if (x.includes("LAND")) return "LANDING";
  if (x.includes("DISARM")) return "DISARMING";
  if (x.includes("MANUAL")) return "MANUAL";
  return "UNKNOWN";
}

export function stageIndex(state?: string): number {
  const n = normalizeControllerState(state);
  const idx = ORDER.indexOf(n as any);
  return idx >= 0 ? idx : 0;
}
```

---

## 8) UI components (read-only)

In Phase A you can reuse the components already shown in the main full-stack guide:

- `StatusCard`
- `MissionStepper`
- `TelemetryCharts` (altitude/speed)
- `TrajectoryPlot` (East vs North)
- `EventLog` (recent events)

The key Phase A requirement is: do **not** include `CommandPanel` or `VoicePanel` yet.

---

## 9) Dashboard wiring

### 9.1 `src/screens/DashboardScreen.tsx`

```tsx
import React, { useEffect } from "react";
import { ScrollView, View } from "react-native";
import { connectStream } from "../api/ws";
import { getStatus } from "../api/http";
import { useAppStore } from "../state/store";
import { StatusCard } from "../components/StatusCard";
import { MissionStepper } from "../components/MissionStepper";
import { TrajectoryPlot } from "../components/TrajectoryPlot";
import { TelemetryCharts } from "../components/TelemetryCharts";
import { EventLog } from "../components/EventLog";

export function DashboardScreen() {
  const ingest = useAppStore((s) => s.ingest);
  const setWsStatus = useAppStore((s) => s.setWsStatus);
  const setSnapshot = useAppStore((s) => s.setSnapshot);

  useEffect(() => {
    void getStatus().then(setSnapshot).catch(() => null);

    const disconnect = connectStream(
      (evt) => ingest(evt),
      (s) => setWsStatus(s),
    );
    return disconnect;
  }, [ingest, setWsStatus, setSnapshot]);

  return (
    <ScrollView contentContainerStyle={{ padding: 12 }}>
      <View style={{ gap: 12 }}>
        <StatusCard />
        <MissionStepper />
        <TrajectoryPlot />
        <TelemetryCharts />
        <EventLog />
      </View>
    </ScrollView>
  );
}
```

---

## 10) `App.tsx` (tabs)

```tsx
import React from "react";
import { NavigationContainer } from "@react-navigation/native";
import { createBottomTabNavigator } from "@react-navigation/bottom-tabs";
import { DashboardScreen } from "./src/screens/DashboardScreen";

const Tab = createBottomTabNavigator();

export default function App() {
  return (
    <NavigationContainer>
      <Tab.Navigator>
        <Tab.Screen name="Dashboard" component={DashboardScreen} />
      </Tab.Navigator>
    </NavigationContainer>
  );
}
```

---

## 11) Phase A acceptance criteria

- UI shows the correct current FSM stage from `controller_status.controller_state`.
- UI plots reflect live `position_sample` telemetry.
- UI displays recent operator actions by streaming `operator_commands` events.
- UI cannot send commands.
