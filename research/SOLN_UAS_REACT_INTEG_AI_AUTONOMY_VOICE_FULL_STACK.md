
# UAS React Native Frontend — Full-Stack Integration Guide (AI Autonomy + Voice)

This guide walks through a **type-it-yourself** React Native (Expo) + TypeScript frontend that integrates with your existing backend infrastructure:

- **ROS2 Jazzy C++** autonomy runtime (controller/vehicle/logger)
- **LeRobot pipeline** (Shadow/Mixed/Policy modes, episode recording, evaluation)
- **MiNiFi C++** sidecar (tails JSONL and ships telemetry/ops)
- **Operator voice stack** (Hugging Face ASR + Rasa dialogue + action server)

The frontend goals:

1) Issue operator commands via **buttons** and **voice**
2) See **status and telemetry** in real time
3) Visualize the mission’s **event-driven FSM workflow** (Manual → Arming → Takeoff → Waypoint → Landing → Disarming)
4) See how voice commands were **transcribed → interpreted → executed**
5) Monitor AI autonomy modes (Classical/Shadow/Mixed/Policy) and intervene safely

Safety boundary (unchanged):

- The UI **never publishes actuation directly**.
- The UI talks only to **control-plane** HTTP endpoints (safe, bounded, validated).
- The authoritative actuation contract stays **`CommandIntent` → `action_arbiter_node` → `vehicle_interface_node`**.

---

## Read this first (quick navigation)

If you want the end-to-end order across Python → C++ → MiNiFi → ROS2 → LeRobot → voice/Rasa → UI, see:

- `SOLUTION_LEARNING_PATH_FULL_STACK_AI_AUTONOMY_VOICE_UI.md`

If you’re building the UI incrementally, follow this order:

1) **Start here (this doc):** `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK.md`
	- Skim sections **0–1** first (backend contracts + Operator Gateway boundary).

2) **Pick your current rollout phase (deep-dive, “type-it-yourself”):**
	- Phase A (read-only dashboard): `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK_PHASEA.md`
	- Phase B (UI buttons + confirmations + audit): `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK_PHASEB.md`
	- Phase C (voice from UI upload): `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK_PHASEC.md`
	- Phase D (run/episode IDs + filtering): `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK_PHASED.md`

3) **Match your repo layout (folder structure references):**
	- Final “full stack” structure (Operator Gateway + mobile UI alongside voice/Rasa/ROS2):
	  `SOLN_DES_DIR_STRUCTURE_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK.md`
	- Incremental structure per rollout phase (A→D deltas):
	  `SOLN_DES_DIR_STRUCTURE_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK_PHASES_A_TO_D.md`

---

## 0) Re-stating the backend contracts the UI must respect

### 0.1 Control-plane HTTP endpoints (already established)

From the voice + ROS2 gateway docs, the operator command vocabulary maps to these localhost endpoints:

- `POST /v1/mission/start` body: `{ "run_id": "...", "vehicle_id": "...", "mode": "classical" }`
- `POST /v1/mission/stop` body: `{ "reason": "operator_request" }`

- `POST /v1/policy/mode` body: `{ "mode": "shadow|mixed|policy", "allow_action_types": [6] }`
	- where `6` corresponds to `CMD_POSITION` in the ROS2 message enum

- `POST /v1/episode/start` body: `{ "episode_id": "..." }`
- `POST /v1/episode/stop` body: `{ "episode_id": "active" }`

- `POST /v1/safety/stop` body: `{ "behavior": "stop|land" }`

### 0.2 Telemetry/status signals the UI should visualize

Your runtime already produces JSONL streams under `Logs/` that MiNiFi tails:

- `Logs/telemetry.jsonl` (events + samples)
	- `state_transition`, `command_waypoint`, `position_sample`, `mission_summary`, ...
- `Logs/controller_status.jsonl` (loop health, controller FSM state, mission flag)
- `Logs/learning_events.jsonl` (policy-mode changes, episode boundaries)
- `Logs/policy_eval.jsonl` (shadow diffs)
- `Logs/operator_commands.jsonl` (voice + UI command audit)

The UI should not need to read JSONL directly; instead it should receive **already-parsed JSON events** via a gateway.

---

## 1) Why you want an “Operator Gateway” between UI and ROS2

Your existing ROS2 HTTP gateway is designed as a **localhost** control-plane seam for MiNiFi and local operator tools.

For a real UI (phone/web/desktop), you need one additional component:

**Operator Gateway (network-facing)**

- Exposes a stable API to UIs (REST + WS)
- Proxies safe commands to the ROS2 localhost gateway (`/v1/*`)
- Streams status/telemetry/operator events by tailing `Logs/*.jsonl` or subscribing to ROS2 topics
- Optionally hosts “voice upload” endpoints (UI records audio; backend runs HF ASR + Rasa)

This keeps the safety boundary intact and solves practical issues:

- Mobile devices cannot access `127.0.0.1` on your ROS2 machine
- Streaming JSONL to a UI is awkward; better to stream structured events over WebSocket
- You can centralize security (API keys), rate limits, and cross-origin settings in one place

### 1.1 Minimal Operator Gateway API for the UI

This guide assumes the UI talks to a base URL, e.g. `http://<operator-gateway-host>:8088`.

REST:

- `GET /api/v1/status` → returns a compact snapshot (controller state, in_mission, policy mode, loop health, last operator cmd)
- `POST /api/v1/cmd/mission/start` → proxies to `/v1/mission/start`
- `POST /api/v1/cmd/mission/stop` → proxies to `/v1/mission/stop`
- `POST /api/v1/cmd/policy/mode` → proxies to `/v1/policy/mode`
- `POST /api/v1/cmd/episode/start` → proxies to `/v1/episode/start`
- `POST /api/v1/cmd/episode/stop` → proxies to `/v1/episode/stop`
- `POST /api/v1/cmd/safety/stop` → proxies to `/v1/safety/stop`

Streaming:

- `WS /api/v1/stream` → pushes parsed events (telemetry/status/learning/policy/operator)

Optional voice (recommended if you want “voice from phone”):

- `POST /api/v1/voice/command` (multipart audio) → runs HF ASR → Rasa → action server → ROS2 gateway; returns transcript + intent + execution

Event envelope (example):

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

## 2) Step-by-step: build the React Native (Expo) app (TypeScript)

### Step 1 — Create an Expo app

From your repo root (or under `operator/ui/`):

```bash
npx create-expo-app operator-ui --template
```

Pick a TypeScript template (Expo provides TS-enabled templates).

Suggested: keep it Expo-managed at first (fast iteration across iOS/Android/Web).

### Step 2 — Add dependencies

From the app folder:

```bash
cd operator-ui

# navigation
npm i @react-navigation/native @react-navigation/bottom-tabs
npx expo install react-native-screens react-native-safe-area-context

# charts / svg
npm i victory-native
npx expo install react-native-svg

# audio recording (for “voice from UI”)
npx expo install expo-av

# tiny state store
npm i zustand
```

Notes:

- `victory-native` uses `react-native-svg` for rendering.
- WebSocket is built into React Native; no extra package required.

### Step 3 — Create a small, explicit `src/` structure

Inside the Expo app:

```text
operator-ui/
	src/
		api/
			config.ts
			http.ts
			types.ts
			ws.ts
		state/
			store.ts
			selectors.ts
		components/
			StatusCard.tsx
			MissionStepper.tsx
			TelemetryCharts.tsx
			TrajectoryPlot.tsx
			CommandPanel.tsx
			VoicePanel.tsx
			EventLog.tsx
		screens/
			DashboardScreen.tsx
			MissionScreen.tsx
			TelemetryScreen.tsx
			OperatorScreen.tsx
			SettingsScreen.tsx
		util/
			time.ts
			math.ts
	App.tsx
```

Keep everything frontend-only. No ROS2/MiNiFi dependencies.

---

## 3) Implement the API layer (TypeScript)

### 3.1 `src/api/config.ts`

Use an environment variable so the same app works on device and on web.

```ts
export const API_BASE_URL =
	process.env.EXPO_PUBLIC_API_BASE_URL?.replace(/\/$/, "") ?? "http://127.0.0.1:8088";

export const WS_URL = API_BASE_URL.replace(/^http/, "ws") + "/api/v1/stream";
```

When running on a phone, `127.0.0.1` refers to the phone. You must set `EXPO_PUBLIC_API_BASE_URL` to your dev machine’s LAN IP.

### 3.2 `src/api/types.ts`

Define the normalized event types your UI expects.

```ts
export type StreamName =
	| "telemetry"
	| "controller_status"
	| "learning_events"
	| "policy_eval"
	| "operator_commands";

export type Ned = { north: number; east: number; down: number };

export type ControllerStatus = {
	controller_state: string; // Manual/Arming/Takeoff/Waypoint/Landing/Disarming
	in_mission: boolean;
	loop_rate_hz: number;
	loop_jitter_ms_p95: number;
	t_us: number;
	vehicle_id: string;
	run_id: string;
};

export type TelemetryEventType =
	| "state_transition"
	| "position_sample"
	| "command_waypoint"
	| "mission_summary";

export type TelemetryEventPayload =
	| { state_from: string; state_to: string }
	| { position_ned: Ned; velocity_ned?: Ned }
	| { target_ned: Ned; heading_rad?: number }
	| { ok: boolean; reason?: string };

export type UiEventEnvelope = {
	stream: StreamName;
	t_us: number;
	vehicle_id: string;
	run_id: string;
	event_type: string;
	payload: Record<string, unknown>;
};

export type StatusSnapshot = {
	controller?: ControllerStatus;
	policy_mode?: "CLASSICAL" | "SHADOW" | "MIXED" | "POLICY";
	episode_id?: string | null;
	last_operator_command?: {
		t_us: number;
		transcript?: string;
		intent?: string;
		ok?: boolean;
	};
};
```

If your gateway produces richer payloads, extend this file; keep UI types stable.

### 3.3 `src/api/http.ts`

Small helper with bounded timeouts.

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
		if (!res.ok) throw new Error(`HTTP ${res.status}`);
		return (await res.json()) as TRes;
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

### 3.4 `src/api/ws.ts`

Use WebSocket for real-time updates.

```ts
import { WS_URL } from "./config";
import type { UiEventEnvelope } from "./types";

export type OnEvent = (evt: UiEventEnvelope) => void;

export function connectStream(onEvent: OnEvent, onStatus?: (s: string) => void): () => void {
	const ws = new WebSocket(WS_URL);
	ws.onopen = () => onStatus?.("connected");
	ws.onclose = () => onStatus?.("disconnected");
	ws.onerror = () => onStatus?.("error");
	ws.onmessage = (msg) => {
		try {
			const evt = JSON.parse(String(msg.data)) as UiEventEnvelope;
			onEvent(evt);
		} catch {
			// ignore malformed
		}
	};
	return () => ws.close();
}
```

---

## 4) State management: store the latest status + small ring buffers

### 4.1 `src/state/store.ts`

The UI wants:

- the **latest** controller status
- a short history of telemetry samples for plots
- the latest operator command status

```ts
import { create } from "zustand";
import type { ControllerStatus, Ned, StatusSnapshot, UiEventEnvelope } from "../api/types";

type Sample = { t_us: number; pos?: Ned; vel?: Ned };

type Store = {
	snapshot: StatusSnapshot;
	controller?: ControllerStatus;
	samples: Sample[];
	events: UiEventEnvelope[];
	wsStatus: "disconnected" | "connected" | "error";

	ingest: (evt: UiEventEnvelope) => void;
	setWsStatus: (s: Store["wsStatus"]) => void;
	setSnapshot: (s: StatusSnapshot) => void;
	clear: () => void;
};

const MAX_SAMPLES = 600; // ~10s @ 60Hz, or ~60s @ 10Hz
const MAX_EVENTS = 200;

export const useAppStore = create<Store>((set, get) => ({
	snapshot: {},
	samples: [],
	events: [],
	wsStatus: "disconnected",

	setWsStatus: (wsStatus) => set({ wsStatus }),
	setSnapshot: (snapshot) => set({ snapshot }),
	clear: () => set({ snapshot: {}, controller: undefined, samples: [], events: [] }),

	ingest: (evt) => {
		const st = get();
		const events = [evt, ...st.events].slice(0, MAX_EVENTS);

		// controller_status stream
		if (evt.stream === "controller_status") {
			const payload = evt.payload as any;
			const controller: ControllerStatus | undefined = payload?.controller_state
				? {
						controller_state: String(payload.controller_state),
						in_mission: Boolean(payload.in_mission),
						loop_rate_hz: Number(payload.loop_rate_hz ?? 0),
						loop_jitter_ms_p95: Number(payload.loop_jitter_ms_p95 ?? 0),
						t_us: Number(payload.t_us ?? evt.t_us),
						vehicle_id: String(payload.vehicle_id ?? evt.vehicle_id),
						run_id: String(payload.run_id ?? evt.run_id),
					}
				: undefined;
			set({ events, controller });
			return;
		}

		// telemetry position samples
		if (evt.stream === "telemetry" && evt.event_type === "position_sample") {
			const p = evt.payload as any;
			const pos = p?.position_ned as Ned | undefined;
			const vel = p?.velocity_ned as Ned | undefined;
			const samples = [{ t_us: evt.t_us, pos, vel }, ...st.samples].slice(0, MAX_SAMPLES);
			set({ events, samples });
			return;
		}

		set({ events });
	},
}));
```

### 4.2 `src/state/selectors.ts`

Derive “workflow stage” from the controller FSM state.

```ts
const ORDER = ["MANUAL", "ARMING", "TAKEOFF", "WAYPOINT", "LANDING", "DISARMING"] as const;

export function normalizeControllerState(s?: string): string {
	if (!s) return "UNKNOWN";
	const x = s.trim().toUpperCase();
	// support both C++/ROS2 names and Python doc names
	if (x.includes("ARM")) return "ARMING";
	if (x.includes("TAKEOFF")) return "TAKEOFF";
	if (x.includes("WAYPOINT")) return "WAYPOINT";
	if (x.includes("LAND")) return "LANDING";
	if (x.includes("DISARM")) return "DISARMING";
	if (x.includes("MANUAL")) return "MANUAL";
	return x;
}

export function stageIndex(state?: string): number {
	const n = normalizeControllerState(state);
	const idx = ORDER.indexOf(n as any);
	return idx >= 0 ? idx : 0;
}
```

---

## 5) Build UI components (workflow + charts + commands)

### 5.1 `src/components/StatusCard.tsx`

```tsx
import React from "react";
import { View, Text } from "react-native";
import { useAppStore } from "../state/store";
import { normalizeControllerState } from "../state/selectors";

export function StatusCard() {
	const controller = useAppStore((s) => s.controller);
	const wsStatus = useAppStore((s) => s.wsStatus);

	return (
		<View style={{ padding: 12, borderWidth: 1, borderRadius: 10 }}>
			<Text style={{ fontWeight: "600" }}>Backend Status</Text>
			<Text>WS: {wsStatus}</Text>
			<Text>Vehicle: {controller?.vehicle_id ?? "-"}</Text>
			<Text>Run: {controller?.run_id ?? "-"}</Text>
			<Text>FSM: {normalizeControllerState(controller?.controller_state)}</Text>
			<Text>In mission: {String(controller?.in_mission ?? false)}</Text>
			<Text>Loop: {controller ? `${controller.loop_rate_hz.toFixed(1)} Hz` : "-"}</Text>
			<Text>Jitter p95: {controller ? `${controller.loop_jitter_ms_p95.toFixed(2)} ms` : "-"}</Text>
		</View>
	);
}
```

### 5.2 `src/components/MissionStepper.tsx`

Minimal “workflow stage” UI for the event-driven FSM.

```tsx
import React from "react";
import { View, Text } from "react-native";
import { useAppStore } from "../state/store";
import { stageIndex } from "../state/selectors";

const STAGES = ["Manual", "Arming", "Takeoff", "Waypoint", "Landing", "Disarming"];

export function MissionStepper() {
	const state = useAppStore((s) => s.controller?.controller_state);
	const idx = stageIndex(state);

	return (
		<View style={{ padding: 12, borderWidth: 1, borderRadius: 10 }}>
			<Text style={{ fontWeight: "600" }}>Mission Workflow</Text>
			{STAGES.map((label, i) => (
				<Text key={label} style={{ opacity: i <= idx ? 1.0 : 0.35 }}>
					{i === idx ? "→ " : "  "}{label}
				</Text>
			))}
		</View>
	);
}
```

### 5.3 `src/components/TelemetryCharts.tsx`

Plot altitude (down), speed, and loop jitter.

```tsx
import React, { useMemo } from "react";
import { View, Text } from "react-native";
import { VictoryChart, VictoryLine, VictoryTheme } from "victory-native";
import { useAppStore } from "../state/store";

export function TelemetryCharts() {
	const samples = useAppStore((s) => s.samples);
	const controller = useAppStore((s) => s.controller);

	const alt = useMemo(() =>
		samples
			.filter((s) => s.pos)
			.slice(0, 200)
			.map((s, i) => ({ x: i, y: -Number(s.pos!.down) })),
	[samples]);

	const speed = useMemo(() =>
		samples
			.filter((s) => s.vel)
			.slice(0, 200)
			.map((s, i) => {
				const v = s.vel!;
				const mps = Math.sqrt(v.north * v.north + v.east * v.east + v.down * v.down);
				return { x: i, y: mps };
			}),
	[samples]);

	return (
		<View style={{ gap: 12 }}>
			<View style={{ padding: 12, borderWidth: 1, borderRadius: 10 }}>
				<Text style={{ fontWeight: "600" }}>Altitude (m)</Text>
				<VictoryChart theme={VictoryTheme.material} height={180} padding={{ top: 20, left: 55, right: 20, bottom: 30 }}>
					<VictoryLine data={alt} />
				</VictoryChart>
			</View>

			<View style={{ padding: 12, borderWidth: 1, borderRadius: 10 }}>
				<Text style={{ fontWeight: "600" }}>Speed (m/s)</Text>
				<VictoryChart theme={VictoryTheme.material} height={180} padding={{ top: 20, left: 55, right: 20, bottom: 30 }}>
					<VictoryLine data={speed} />
				</VictoryChart>
			</View>

			<View style={{ padding: 12, borderWidth: 1, borderRadius: 10 }}>
				<Text style={{ fontWeight: "600" }}>Control Loop Jitter p95 (ms)</Text>
				<Text>{controller ? controller.loop_jitter_ms_p95.toFixed(2) : "-"}</Text>
			</View>
		</View>
	);
}
```

### 5.4 `src/components/TrajectoryPlot.tsx`

Plot the 10m box path in N/E.

```tsx
import React, { useMemo } from "react";
import { View, Text } from "react-native";
import Svg, { Polyline } from "react-native-svg";
import { useAppStore } from "../state/store";

export function TrajectoryPlot() {
	const samples = useAppStore((s) => s.samples);
	const points = useMemo(() => {
		const pts = samples
			.filter((s) => s.pos)
			.slice(0, 400)
			.map((s) => s.pos!);
		if (pts.length < 2) return "";

		const ns = pts.map((p) => p.north);
		const es = pts.map((p) => p.east);
		const nMin = Math.min(...ns);
		const nMax = Math.max(...ns);
		const eMin = Math.min(...es);
		const eMax = Math.max(...es);
		const pad = 0.0001;
		const scale = (x: number, lo: number, hi: number, outLo: number, outHi: number) =>
			outLo + ((x - lo) / (hi - lo + pad)) * (outHi - outLo);

		return pts
			.map((p) => {
				const x = scale(p.east, eMin, eMax, 10, 290);
				const y = 300 - scale(p.north, nMin, nMax, 10, 290);
				return `${x.toFixed(1)},${y.toFixed(1)}`;
			})
			.join(" ");
	}, [samples]);

	return (
		<View style={{ padding: 12, borderWidth: 1, borderRadius: 10 }}>
			<Text style={{ fontWeight: "600" }}>Trajectory (East vs North)</Text>
			<Svg width={300} height={300}>
				<Polyline points={points} fill="none" strokeWidth={2} stroke="#333" />
			</Svg>
		</View>
	);
}
```

If your repo uses a theme system, swap hard-coded colors for theme tokens.

### 5.5 `src/components/CommandPanel.tsx`

Buttons issue control-plane requests (same semantics as voice).

```tsx
import React, { useState } from "react";
import { View, Text, Pressable } from "react-native";
import { api } from "../api/http";

function Btn({ label, onPress }: { label: string; onPress: () => Promise<void> }) {
	return (
		<Pressable
			onPress={() => void onPress()}
			style={{ padding: 10, borderWidth: 1, borderRadius: 10, alignItems: "center" }}
		>
			<Text>{label}</Text>
		</Pressable>
	);
}

export function CommandPanel() {
	const [vehicleId] = useState("sim-01");
	const [runId] = useState(() => new Date().toISOString());

	return (
		<View style={{ gap: 10, padding: 12, borderWidth: 1, borderRadius: 10 }}>
			<Text style={{ fontWeight: "600" }}>Commands</Text>
			<Btn label="Start Mission (Classical)" onPress={() => api.missionStart(runId, vehicleId).then(() => Promise.resolve())} />
			<Btn label="Stop Mission" onPress={() => api.missionStop()} />
			<Btn label="Policy Mode: Shadow" onPress={() => api.policyMode("shadow")} />
			<Btn label="Policy Mode: Mixed (CMD_POSITION)" onPress={() => api.policyMode("mixed")} />
			<Btn label="Policy Mode: Policy (CMD_POSITION)" onPress={() => api.policyMode("policy")} />
			<Btn label="Episode Start" onPress={() => api.episodeStart(`ep-${Math.random().toString(16).slice(2, 8)}`)} />
			<Btn label="Episode Stop" onPress={() => api.episodeStop()} />
			<Btn label="EMERGENCY: Land" onPress={() => api.safetyStop("land")} />
		</View>
	);
}
```

For Mixed/Policy mode, you should add a confirmation dialog in the UI (same principle as the Rasa rules).

### 5.6 `src/components/VoicePanel.tsx` (optional, but matches your request)

This is how you make “voice from phone/web UI” work while still using **HF ASR + Rasa** on the backend.

Assumption: your Operator Gateway implements `POST /api/v1/voice/command` and returns JSON:

```json
{ "transcript": "...", "intent": "...", "ok": true, "http_status": 200 }
```

```tsx
import React, { useRef, useState } from "react";
import { View, Text, Pressable } from "react-native";
import { Audio } from "expo-av";
import { API_BASE_URL } from "../api/config";

export function VoicePanel() {
	const recordingRef = useRef<Audio.Recording | null>(null);
	const [status, setStatus] = useState<string>("idle");
	const [last, setLast] = useState<string>("");

	async function start() {
		setStatus("recording");
		await Audio.requestPermissionsAsync();
		await Audio.setAudioModeAsync({ allowsRecordingIOS: true, playsInSilentModeIOS: true });
		const rec = new Audio.Recording();
		await rec.prepareToRecordAsync(Audio.RecordingOptionsPresets.HIGH_QUALITY);
		await rec.startAsync();
		recordingRef.current = rec;
	}

	async function stopAndSend() {
		const rec = recordingRef.current;
		if (!rec) return;
		setStatus("uploading");
		await rec.stopAndUnloadAsync();
		const uri = rec.getURI();
		recordingRef.current = null;
		if (!uri) throw new Error("No audio uri");

		const form = new FormData();
		form.append("audio", {
			// @ts-expect-error React Native file shape
			uri,
			name: "command.m4a",
			type: "audio/m4a",
		});

		const res = await fetch(API_BASE_URL + "/api/v1/voice/command", { method: "POST", body: form });
		const json = await res.json();
		setLast(JSON.stringify(json));
		setStatus("idle");
	}

	return (
		<View style={{ gap: 10, padding: 12, borderWidth: 1, borderRadius: 10 }}>
			<Text style={{ fontWeight: "600" }}>Voice (HF ASR + Rasa)</Text>
			<Text>Status: {status}</Text>
			<Pressable onPress={() => void start()} style={{ padding: 10, borderWidth: 1, borderRadius: 10 }}>
				<Text>Start Recording</Text>
			</Pressable>
			<Pressable onPress={() => void stopAndSend()} style={{ padding: 10, borderWidth: 1, borderRadius: 10 }}>
				<Text>Stop + Send</Text>
			</Pressable>
			<Text numberOfLines={6}>Last: {last}</Text>
		</View>
	);
}
```

If you want push-to-talk UX, map “press in” to start and “press out” to stop.

### 5.7 `src/components/EventLog.tsx`

Show recent events including operator commands.

```tsx
import React from "react";
import { View, Text, ScrollView } from "react-native";
import { useAppStore } from "../state/store";

export function EventLog() {
	const events = useAppStore((s) => s.events);
	return (
		<View style={{ padding: 12, borderWidth: 1, borderRadius: 10 }}>
			<Text style={{ fontWeight: "600" }}>Recent Events</Text>
			<ScrollView style={{ maxHeight: 240 }}>
				{events.map((e, i) => (
					<Text key={i} numberOfLines={2}>
						[{e.stream}] {e.event_type}
					</Text>
				))}
			</ScrollView>
		</View>
	);
}
```

---

## 6) Screens + navigation

### 6.1 `src/screens/DashboardScreen.tsx`

```tsx
import React, { useEffect } from "react";
import { ScrollView, View } from "react-native";
import { connectStream } from "../api/ws";
import { api } from "../api/http";
import { useAppStore } from "../state/store";
import { StatusCard } from "../components/StatusCard";
import { MissionStepper } from "../components/MissionStepper";
import { CommandPanel } from "../components/CommandPanel";
import { EventLog } from "../components/EventLog";

export function DashboardScreen() {
	const ingest = useAppStore((s) => s.ingest);
	const setWsStatus = useAppStore((s) => s.setWsStatus);
	const setSnapshot = useAppStore((s) => s.setSnapshot);

	useEffect(() => {
		// initial snapshot (best effort)
		void api.status().then((s: any) => setSnapshot(s)).catch(() => null);

		const disconnect = connectStream(
			(evt) => ingest(evt),
			(s) => setWsStatus(s === "connected" ? "connected" : s === "error" ? "error" : "disconnected"),
		);
		return disconnect;
	}, [ingest, setWsStatus, setSnapshot]);

	return (
		<ScrollView contentContainerStyle={{ padding: 12 }}>
			<View style={{ gap: 12 }}>
				<StatusCard />
				<MissionStepper />
				<CommandPanel />
				<EventLog />
			</View>
		</ScrollView>
	);
}
```

### 6.2 `src/screens/MissionScreen.tsx`

Focus: workflow, state transitions, mission summary.

- Reuse `MissionStepper`
- Add a timeline view keyed off `telemetry.state_transition`

### 6.3 `src/screens/TelemetryScreen.tsx`

Focus: plots.

- `TrajectoryPlot`
- `TelemetryCharts`
- If you also stream `policy_eval`, add a plot for `err_n/err_e/err_d` percentiles.

### 6.4 `src/screens/OperatorScreen.tsx`

Focus: voice + operator commands log.

- `VoicePanel`
- filtered `EventLog` for `operator_commands` stream

### 6.5 `App.tsx` (bottom tabs)

```tsx
import React from "react";
import { NavigationContainer } from "@react-navigation/native";
import { createBottomTabNavigator } from "@react-navigation/bottom-tabs";
import { DashboardScreen } from "./src/screens/DashboardScreen";
import { TelemetryScreen } from "./src/screens/TelemetryScreen";
import { OperatorScreen } from "./src/screens/OperatorScreen";

const Tab = createBottomTabNavigator();

export default function App() {
	return (
		<NavigationContainer>
			<Tab.Navigator>
				<Tab.Screen name="Dashboard" component={DashboardScreen} />
				<Tab.Screen name="Telemetry" component={TelemetryScreen} />
				<Tab.Screen name="Operator" component={OperatorScreen} />
			</Tab.Navigator>
		</NavigationContainer>
	);
}
```

---

## 7) What to visualize (high-value telemetry views)

This section ties directly to your existing JSONL contracts.

### 7.1 Mission workflow (FSM-driven)

Source signals:

- `controller_status.controller_state` (current)
- `telemetry.state_transition` events (history)

UI views:

- Current stage stepper (Manual → ... → Disarming)
- Timeline of transitions with timestamps
- “Time in stage” counters (duration from transition events)

### 7.2 Flight telemetry (NED)

Source signals:

- `telemetry.position_sample.position_ned` and `velocity_ned`

UI views:

- Trajectory plot (East vs North)
- Altitude over time (use $-down$)
- Speed magnitude over time

### 7.3 Controller health

Source signals:

- `controller_status.loop_rate_hz`
- `controller_status.loop_jitter_ms_p95`

UI views:

- Jitter trend (if you stream status periodically)
- “Healthy / degraded” indicator with thresholds

### 7.4 LeRobot autonomy mode and policy quality

Source signals:

- `learning_events.jsonl` mode changes and episode boundaries
- `policy_eval.jsonl` shadow diffs

UI views:

- Mode badge: Classical / Shadow / Mixed / Policy
- Shadow error plots (`err_n`, `err_e`, `err_d`) over time
- Mixed/Policy: “arbiter fallback rate” if you emit it

### 7.5 Operator commands (voice + buttons)

Source signals:

- `operator_commands.jsonl`

UI views:

- Transcript
- Parsed intent
- Confirmation prompts (if routed through Rasa)
- Execution result (HTTP status, ok/error)

This is the “glass box” view that answers: *what did the operator say? what did the system do?*

---

## 8) Step-by-step transition: CLI/voice-only → UI-driven ops

This is a safe migration that preserves your current workflow.

### Phase A — UI as read-only dashboard

Deep-dive implementation guide:

- `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK_PHASEA.md`

1) Keep using CLI/voice to start missions.
2) Add Operator Gateway streaming `controller_status` + `telemetry`.
3) UI shows workflow + plots.

Success criteria:

- UI displays the correct FSM stage progression while the Unity mission runs.

### Phase B — Add UI buttons for the same `/v1/*` commands

Deep-dive implementation guide:

- `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK_PHASEB.md`

1) Implement `CommandPanel` (start/stop/mode/episode/safety).
2) Gate risky actions with confirmation dialogs.
3) Make sure every button press appends an entry into `operator_commands.jsonl` (gateway-side audit).

Success criteria:

- Starting/stopping mission from UI is equivalent to CLI.

### Phase C — Add “voice from UI” (HF ASR + Rasa)

Deep-dive implementation guide:

- `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK_PHASEC.md`

1) UI records a short clip.
2) UI uploads audio to the Operator Gateway.
3) Gateway runs HF ASR → sends transcript to Rasa → action server calls ROS2 gateway.
4) Gateway logs the attempt to `operator_commands.jsonl` and streams the result back.

Success criteria:

- Operator can use phone voice to run the same commands as before.

### Phase D — Tie UI workflow to run/episode IDs

Deep-dive implementation guide:

- `SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK_PHASED.md`

1) UI generates and displays `run_id` (and optionally `episode_id`).
2) UI filters events by `(vehicle_id, run_id)`.
3) UI provides “export run” convenience links (to where logs/datasets live).

---

## 9) Things to be careful about (practical + safety)

1) **Do not expose ROS2 DDS to the UI.** Keep UI → Gateway (HTTP/WS) → ROS2 gateway (localhost).
2) **Keep Mixed/Policy guarded.** Confirmations in UI and Rasa. Arbiter remains authoritative.
3) **Rate-limit operator commands.** UI and gateway should debounce mode flips.
4) **Network reality for mobile.** Use LAN IP, handle reconnects, show “stale data” clearly.
5) **Time sync.** Prefer microsecond timestamps (`t_us`) and show relative time in the UI.

---

## 10) Next implementation step (recommended)

If you want the UI to work on a phone immediately, the next concrete backend work item is:

- implement the **Operator Gateway** WS stream that emits parsed events from the `Logs/*.jsonl` files

Then the UI in this guide becomes functional without changing ROS2 or MiNiFi.

