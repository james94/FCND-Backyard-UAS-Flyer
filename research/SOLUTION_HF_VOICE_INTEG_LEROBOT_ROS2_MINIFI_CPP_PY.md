
# Backyard Flyer — Hugging Face Voice Commands Integration (LeRobot + ROS2 Jazzy + MiNiFi, Python)

This document extends the existing **end-to-end AI autonomy pipeline** (LeRobot/`lerobot-ros` + ROS2 Jazzy C++ + MiNiFi C++) by adding an **operator voice interface**.

Goal:

- After you launch the backend system (ROS2 nodes + optional MiNiFi sidecar), an operator can issue **high-level commands** by speaking (instead of typing in a terminal).
- Commands can start/stop the mission, switch autonomy modes (Shadow/Mixed/Policy), start/stop episode recording, and request an emergency stop.

Non-goals / safety boundary (consistent with the existing docs):

- **Voice recognition is not in the actuation loop.**
- The **authoritative actuation contract stays** `CommandIntent` routed through `action_arbiter_node` → `vehicle_interface_node`.
- MiNiFi remains a **telemetry + ops/orchestration sidecar**, not a real-time dependency.

This writeup is intentionally compatible with:

- `SOLUTION_LEROBOT_ROS2_JAZZY_CPP.md` (Part 1: learning contracts + nodes)
- `SOLUTION_LEROBOT_ROS2_JAZZY_CPP_PY_PART2.md` (Part 2: dataset export/training/policy modes)
- `SOLUTION_ROS2_JAZZY_CPP.md` (ROS2 node split + HTTP gateway concept)
- `SOLUTION_MINIFI_CPP.md` and `SOLUTION_MINIFI_CPP_V2.md` (JSONL telemetry + HTTP control-plane boundary)

---

## 0) Why voice commands fit the existing architecture

You already have (or have explicitly planned) the correct seam for “operator commands”:

- `ros2_http_gateway_node` exposes **localhost** endpoints like `POST /v1/mission/start`, `POST /v1/mission/stop`, and (in the LeRobot docs) `POST /v1/policy/mode`.
- Those endpoints translate into ROS2 service calls / parameter updates that are applied at safe boundaries inside ROS2.

Voice commands become just another client of that control-plane:

```
[Operator] --(audio)--> [HF ASR + command parser (Python)] --(HTTP localhost)--> [ros2_http_gateway_node]
																																					|
																																					+--> Logs/operator_commands.jsonl (MiNiFi tails)
```

Key property:

- If the voice service crashes, the vehicle still flies (or stops) based on the classical controller + arbiter + vehicle node.

---

## 1) “CLI → Voice” transition, step-by-step (MVP to production-ish)

### Step 1 — Freeze a small, explicit command vocabulary (no free-form actuation)

Start with a constrained set of high-level operator intents. Example MVP list:

1) Mission lifecycle
- `MISSION_START_CLASSICAL`
- `MISSION_STOP`

2) Policy modes (mapped to your Part 2 rollout)
- `POLICY_MODE_SHADOW`
- `POLICY_MODE_MIXED_CMD_POSITION_ONLY`
- `POLICY_MODE_POLICY_CMD_POSITION_ONLY`

3) Episode/data collection
- `EPISODE_START`
- `EPISODE_STOP`

4) Safety
- `EMERGENCY_STOP` (maps to a safe stop/land sequence; keep this *always available*)

Why this matters:

- It prevents the voice layer from becoming an unbounded “natural language → actuation” translator.
- It keeps safety logic in the same place you already designed: the ROS2 controller + arbiter.

### Step 2 — Ensure your control-plane endpoints cover the vocabulary

For the MVP, the voice service should only call a small set of localhost endpoints.

Recommended endpoints (consistent with your existing docs):

- `POST /v1/mission/start` body: `{ "run_id": "...", "vehicle_id": "...", "mode": "classical" }`
- `POST /v1/mission/stop` body: `{ "reason": "operator_request" }`

- `POST /v1/policy/mode` body: `{ "mode": "shadow|mixed|policy", "allow_action_types": [6] }`
	- where `6` corresponds to `CMD_POSITION` in `CommandIntent.msg` from `SOLUTION_ROS2_JAZZY_CPP.md`

- `POST /v1/episode/start` body: `{ "episode_id": "..." }`
- `POST /v1/episode/stop` body: `{ "episode_id": "..." }`

- `POST /v1/safety/stop` body: `{ "behavior": "stop|land" }`

Notes:

- These endpoints are **ops/control-plane**. The *actuation* still happens via the normal ROS2 data plane.
- Handlers must remain non-blocking (handoff into a queue / apply on a safe boundary), per `SOLUTION_MINIFI_CPP_V2.md`.

### Step 3 — Add an “Operator Command” JSONL log stream (for audit + UI)

Create a new append-only log file written by the voice service:

- `Logs/operator_commands.jsonl`

Each line is one command attempt:

```json
{
	"event_type": "operator_command",
	"t_us": 1700000001123456,
	"vehicle_id": "sim-01",
	"run_id": "2026-02-24T01-23-45Z",

	"input": {
		"transcript": "start the mission in shadow mode",
		"audio": {"sample_rate_hz": 16000, "duration_ms": 2100}
	},

	"decision": {
		"intent": "POLICY_MODE_SHADOW",
		"confidence": 0.86,
		"requires_confirm": false
	},

	"execution": {
		"target": "http://127.0.0.1:8080/v1/policy/mode",
		"request": {"mode": "shadow", "allow_action_types": [6]},
		"http_status": 200,
		"ok": true,
		"error": null
	}
}
```

This is intentionally aligned with the rest of your JSONL telemetry strategy:

- MiNiFi can `TailFile` it.
- NiFi / dashboards can correlate it to `learning_events.jsonl`, `policy_eval.jsonl`, and `controller_status.jsonl`.

### Step 4 — Implement audio capture (start with push-to-talk)

For an MVP, do not start with always-on listening.

Recommended MVP behavior:

- Operator presses Enter (or holds a key) to record a short clip.
- The service records up to $N$ seconds (e.g., 3–5s), then runs ASR.

Later (optional): add VAD (voice activity detection) to auto-stop recording.

### Step 5 — Speech-to-text using Hugging Face (ASR)

Use a local/offline ASR model downloaded from Hugging Face.

Two common approaches:

1) `transformers` pipeline (simple; may be heavier)
2) Faster Whisper runtimes (often lower latency), still using HF-hosted Whisper checkpoints

MVP example (pseudo-code):

```python
from transformers import pipeline

asr = pipeline(
		task="automatic-speech-recognition",
		model="openai/whisper-small",
		device="cpu",  # or "cuda"
)

text = asr("/tmp/command.wav")["text"]
```

Operational notes:

- Expect different latency on CPU vs GPU.
- Keep audio preprocessing consistent (mono, 16kHz is typical).

### Step 6 — Text-to-intent parsing (NLU): start rule-based, then (optionally) HF classification

For safety, start with a **deterministic** mapping from transcript → intent.

Example approach:

- Normalize text: lowercase, remove punctuation.
- Match a small set of phrases/keywords.
- If ambiguous, ask for a retry or explicit confirmation.

Example phrases:

- “start mission classical” → `MISSION_START_CLASSICAL`
- “shadow mode” → `POLICY_MODE_SHADOW`
- “mixed mode cmd position” → `POLICY_MODE_MIXED_CMD_POSITION_ONLY`
- “policy mode cmd position” → `POLICY_MODE_POLICY_CMD_POSITION_ONLY`
- “stop mission”, “abort”, “emergency stop” → `EMERGENCY_STOP`

If you later want ML-backed NLU, keep the *output space* identical (same finite intents):

- HF `zero-shot-classification` can map text into one of the allowed intents.
- A small fine-tuned classifier can do the same.

Important: do not let NLU output arbitrary parameters that can bypass `action_arbiter_node` clamps.

### Step 7 — Add a safety gate: confirmations + state-aware checks

Voice interfaces are error-prone. Add guardrails at the operator-command layer:

- Require confirmation for risky transitions (e.g., switching to `POLICY` mode).
- Prevent nonsensical actions (e.g., `EPISODE_START` when mission not running).
- Rate-limit commands (e.g., one mode change per 2 seconds).

Where to enforce what:

- **Voice service**: confirmation UX + debouncing.
- **ROS2 backend**: authoritative state machine + rejection of invalid transitions.
- **Arbiter**: strict timeouts/clamps/safety overrides (already in Part 2).

### Step 8 — Execute by calling localhost HTTP endpoints

Once you have a canonical intent, call the gateway.

MVP pseudo-code:

```python
import requests

def set_shadow_mode():
		r = requests.post(
				"http://127.0.0.1:8080/v1/policy/mode",
				json={"mode": "shadow", "allow_action_types": [6]},
				timeout=1.0,
		)
		r.raise_for_status()
```

Make the timeout short and handle errors cleanly.

### Step 9 — Provide operator feedback (MVP: stdout + JSONL; later: UI)

MVP feedback loop:

- Print: “Heard: …; Parsed: …; Executed: … (200 OK)”
- Append the record to `Logs/operator_commands.jsonl`

Later:

- Speak back using local TTS.
- Show status in a frontend UI.

---

## 2) How the HF voice service interacts with the autonomy pipeline (end-to-end view)

### 2.1 Data plane vs control plane vs telemetry plane

Keep the same plane separation you already established:

1) **Actuation / data plane (real-time-ish, C++)**
- `action_arbiter_node` publishes authoritative `vehicle/command_intent`
- `vehicle_interface_node` executes the intent

2) **Control plane (ops, eventual)**
- `ros2_http_gateway_node` exposes localhost endpoints
- voice service calls these endpoints
- MiNiFi may also call these endpoints (automation), but voice is the human-in-the-loop path

3) **Telemetry plane (observability, eventual)**
- JSONL logs: `telemetry.jsonl`, `controller_status.jsonl`, `learning_events.jsonl`, `policy_eval.jsonl`
- new JSONL: `operator_commands.jsonl`
- MiNiFi tails + forwards

### 2.2 Shadow/Mixed/Policy mode mapping (voice commands)

Your Part 2 progression becomes a voice-driven operator workflow:

1) “Start mission classical”
- `POST /v1/mission/start` with `mode=classical`
- recorder logs raw episodes

2) “Enable shadow mode”
- `POST /v1/policy/mode` → `shadow`
- policy runner publishes candidate actions
- arbiter continues to execute classical intents
- `policy_eval.jsonl` records diffs

3) “Enable mixed mode cmd position only”
- `POST /v1/policy/mode` → `mixed`, `allow_action_types=[CMD_POSITION]`
- arbiter may select policy for CMD_POSITION only (subject to timeout/clamps)
- STOP/LAND always remains classical safety override

4) “Enable policy mode cmd position only”
- `POST /v1/policy/mode` → `policy`, `allow_action_types=[CMD_POSITION]`
- arbiter selects policy actions when fresh; otherwise falls back

5) “Emergency stop”
- `POST /v1/safety/stop` (or `POST /v1/mission/stop`) with an immediate safe behavior

---

## 3) Part 2 — Step-by-step Python implementation (HF ASR → intent → HTTP gateway)

This section is a concrete follow-along for writing the MVP Python voice service described earlier.

Design constraints (carry-over from the ROS2/LeRobot/MiNiFi docs):

- The voice service issues **only control-plane requests** (HTTP localhost).
- The voice service never publishes actuation directly.
- The allowed output is a **finite intent vocabulary**.
- We log every attempt to `Logs/operator_commands.jsonl`.

### 3.1 Create the minimal folder skeleton

Use the structure from `SOLN_DES_DIR_STRUCTURE_HF_VOICE_INTEG_LEROBOT_ROS2_MINIFI_CPP_PY.md`:

```text
operator/
  voice/
	requirements.txt
	app/
	  voice_service.py
	  audio_capture.py
	  hf_asr.py
	  intent_parser.py
	  http_client.py
	  jsonl_logger.py
```

This keeps the voice tool decoupled from ROS2 (HTTP only).

### 3.2 `requirements.txt` (MVP)

Create `operator/voice/requirements.txt`:

```text
requests>=2.31
sounddevice>=0.4.6
soundfile>=0.12.1
numpy>=1.26

torch>=2.1
transformers>=4.38
accelerate>=0.27
```

Notes:

- If you do not want PyTorch in the operator laptop environment, you can swap ASR to an external process/server later.

### 3.3 `audio_capture.py` (push-to-talk recording)

Create `operator/voice/app/audio_capture.py`:

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np
import sounddevice as sd


@dataclass(frozen=True)
class AudioClip:
	samples: np.ndarray  # float32, shape=(n,)
	sample_rate_hz: int

	@property
	def duration_ms(self) -> int:
		return int(1000.0 * (self.samples.shape[0] / float(self.sample_rate_hz)))


def record_push_to_talk(max_seconds: float = 4.0, sample_rate_hz: int = 16000) -> AudioClip:
	"""Blocks until it records a fixed-length clip.

	MVP UX: press Enter to start recording; it records max_seconds and stops.
	"""
	n = int(max_seconds * sample_rate_hz)
	audio = sd.rec(frames=n, samplerate=sample_rate_hz, channels=1, dtype="float32")
	sd.wait()
	mono = np.squeeze(audio, axis=1)
	return AudioClip(samples=mono, sample_rate_hz=sample_rate_hz)
```

### 3.4 `hf_asr.py` (Hugging Face speech-to-text)

Create `operator/voice/app/hf_asr.py`:

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from transformers import pipeline


@dataclass
class AsrResult:
	text: str
	# Some pipelines return extra fields; we keep MVP minimal.


class HfAsr:
	def __init__(self, model: str = "openai/whisper-small", device: str = "cpu") -> None:
		self._pipe = pipeline(
			task="automatic-speech-recognition",
			model=model,
			device=device,
		)

	def transcribe(self, samples: np.ndarray, sample_rate_hz: int) -> AsrResult:
		# transformers ASR pipeline accepts raw arrays for Whisper-like models.
		out = self._pipe({"raw": samples, "sampling_rate": sample_rate_hz})
		text = (out.get("text") or "").strip()
		return AsrResult(text=text)
```

Operational note:

- On some environments, the pipeline may require `ffmpeg` when reading from files; using raw arrays often avoids that.

### 3.5 `intent_parser.py` (deterministic transcript → intent)

Create `operator/voice/app/intent_parser.py`:

```python
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class Intent(str, Enum):
	MISSION_START_CLASSICAL = "MISSION_START_CLASSICAL"
	MISSION_STOP = "MISSION_STOP"
	POLICY_MODE_SHADOW = "POLICY_MODE_SHADOW"
	POLICY_MODE_MIXED_CMD_POSITION_ONLY = "POLICY_MODE_MIXED_CMD_POSITION_ONLY"
	POLICY_MODE_POLICY_CMD_POSITION_ONLY = "POLICY_MODE_POLICY_CMD_POSITION_ONLY"
	EPISODE_START = "EPISODE_START"
	EPISODE_STOP = "EPISODE_STOP"
	EMERGENCY_STOP = "EMERGENCY_STOP"


@dataclass(frozen=True)
class ParsedIntent:
	intent: Intent
	confidence: float
	requires_confirm: bool = False


def parse_intent_rule_based(transcript: str) -> ParsedIntent | None:
	t = " ".join(transcript.lower().strip().split())
	if not t:
		return None

	# Always prioritize safety.
	if any(k in t for k in ["emergency", "abort", "kill", "stop now", "e-stop"]):
		return ParsedIntent(Intent.EMERGENCY_STOP, confidence=0.99, requires_confirm=False)

	if "stop mission" in t or t == "stop" or "end mission" in t:
		return ParsedIntent(Intent.MISSION_STOP, confidence=0.95)

	if "start" in t and "mission" in t and "classical" in t:
		return ParsedIntent(Intent.MISSION_START_CLASSICAL, confidence=0.9)

	if "shadow" in t and "mode" in t:
		return ParsedIntent(Intent.POLICY_MODE_SHADOW, confidence=0.9)

	if "mixed" in t and "mode" in t and ("cmd position" in t or "position" in t):
		return ParsedIntent(Intent.POLICY_MODE_MIXED_CMD_POSITION_ONLY, confidence=0.9, requires_confirm=True)

	if "policy" in t and "mode" in t and ("cmd position" in t or "position" in t):
		return ParsedIntent(Intent.POLICY_MODE_POLICY_CMD_POSITION_ONLY, confidence=0.9, requires_confirm=True)

	if "start" in t and "episode" in t:
		return ParsedIntent(Intent.EPISODE_START, confidence=0.9)

	if "stop" in t and "episode" in t:
		return ParsedIntent(Intent.EPISODE_STOP, confidence=0.9)

	return None
```

Why rule-based first:

- It is predictable and reviewable.
- It prevents “creative” text interpretations from producing unsafe behavior.

### 3.6 `http_client.py` (call ROS2 HTTP gateway)

Create `operator/voice/app/http_client.py`:

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional

import requests


@dataclass(frozen=True)
class HttpResult:
	ok: bool
	status_code: int
	error: str | None


class Ros2GatewayClient:
	def __init__(self, base_url: str = "http://127.0.0.1:8080") -> None:
		self._base_url = base_url.rstrip("/")

	def post(self, path: str, payload: Dict[str, Any], timeout_s: float = 1.0) -> HttpResult:
		url = f"{self._base_url}{path}"
		try:
			r = requests.post(url, json=payload, timeout=timeout_s)
			if 200 <= r.status_code < 300:
				return HttpResult(ok=True, status_code=r.status_code, error=None)
			return HttpResult(ok=False, status_code=r.status_code, error=r.text[:500])
		except Exception as e:  # MVP: collapse exceptions into a string
			return HttpResult(ok=False, status_code=0, error=str(e))
```

### 3.7 `jsonl_logger.py` (append-only audit log)

Create `operator/voice/app/jsonl_logger.py`:

```python
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from typing import Any, Dict


def now_us() -> int:
	return int(time.time() * 1_000_000)


class JsonlLogger:
	def __init__(self, path: str) -> None:
		self._path = path
		os.makedirs(os.path.dirname(path), exist_ok=True)

	def append(self, obj: Dict[str, Any]) -> None:
		with open(self._path, "a", encoding="utf-8") as f:
			f.write(json.dumps(obj) + "\n")
```

### 3.8 `voice_service.py` (wire it all together)

Create `operator/voice/app/voice_service.py`:

```python
from __future__ import annotations

import argparse
import uuid

from audio_capture import record_push_to_talk
from hf_asr import HfAsr
from http_client import Ros2GatewayClient
from intent_parser import Intent, parse_intent_rule_based
from jsonl_logger import JsonlLogger, now_us


CMD_POSITION = 6  # must match backyard_flyer_msgs/CommandIntent.msg


def main() -> None:
	ap = argparse.ArgumentParser()
	ap.add_argument("--vehicle_id", default="sim-01")
	ap.add_argument("--run_id", default=None)
	ap.add_argument("--gateway", default="http://127.0.0.1:8080")
	ap.add_argument("--asr_model", default="openai/whisper-small")
	ap.add_argument("--asr_device", default="cpu")
	ap.add_argument("--max_record_s", type=float, default=4.0)
	args = ap.parse_args()

	run_id = args.run_id or f"voice-{uuid.uuid4().hex[:8]}"
	log = JsonlLogger(path="Logs/operator_commands.jsonl")
	asr = HfAsr(model=args.asr_model, device=args.asr_device)
	gateway = Ros2GatewayClient(base_url=args.gateway)

	print("Voice service ready. Press Enter to record; Ctrl+C to exit.")
	while True:
		input("\n[push-to-talk] Press Enter to record... ")
		clip = record_push_to_talk(max_seconds=args.max_record_s)
		asr_res = asr.transcribe(clip.samples, clip.sample_rate_hz)
		transcript = asr_res.text
		parsed = parse_intent_rule_based(transcript)

		event = {
			"event_type": "operator_command",
			"t_us": now_us(),
			"vehicle_id": args.vehicle_id,
			"run_id": run_id,
			"input": {
				"transcript": transcript,
				"audio": {"sample_rate_hz": clip.sample_rate_hz, "duration_ms": clip.duration_ms},
			},
			"decision": None,
			"execution": None,
		}

		if parsed is None:
			event["decision"] = {"intent": None, "confidence": 0.0, "requires_confirm": False}
			event["execution"] = {"ok": False, "http_status": 0, "error": "unrecognized_intent"}
			log.append(event)
			print(f"Heard: {transcript!r} -> (unrecognized).")
			continue

		if parsed.requires_confirm:
			confirm = input(f"Parsed {parsed.intent}. Type 'yes' to confirm: ").strip().lower()
			if confirm != "yes":
				event["decision"] = {"intent": parsed.intent, "confidence": parsed.confidence, "requires_confirm": True}
				event["execution"] = {"ok": False, "http_status": 0, "error": "operator_cancelled"}
				log.append(event)
				print("Cancelled.")
				continue

		# Map intent -> HTTP request
		if parsed.intent == Intent.MISSION_START_CLASSICAL:
			path, payload = "/v1/mission/start", {"vehicle_id": args.vehicle_id, "run_id": run_id, "mode": "classical"}
		elif parsed.intent == Intent.MISSION_STOP:
			path, payload = "/v1/mission/stop", {"reason": "operator_request"}
		elif parsed.intent == Intent.POLICY_MODE_SHADOW:
			path, payload = "/v1/policy/mode", {"mode": "shadow", "allow_action_types": [CMD_POSITION]}
		elif parsed.intent == Intent.POLICY_MODE_MIXED_CMD_POSITION_ONLY:
			path, payload = "/v1/policy/mode", {"mode": "mixed", "allow_action_types": [CMD_POSITION]}
		elif parsed.intent == Intent.POLICY_MODE_POLICY_CMD_POSITION_ONLY:
			path, payload = "/v1/policy/mode", {"mode": "policy", "allow_action_types": [CMD_POSITION]}
		elif parsed.intent == Intent.EPISODE_START:
			path, payload = "/v1/episode/start", {"episode_id": f"ep-{uuid.uuid4().hex[:6]}"}
		elif parsed.intent == Intent.EPISODE_STOP:
			path, payload = "/v1/episode/stop", {"episode_id": "active"}
		elif parsed.intent == Intent.EMERGENCY_STOP:
			path, payload = "/v1/safety/stop", {"behavior": "land"}
		else:
			path, payload = "/v1/mission/stop", {"reason": "unknown_intent"}

		res = gateway.post(path, payload)
		event["decision"] = {"intent": parsed.intent, "confidence": parsed.confidence, "requires_confirm": parsed.requires_confirm}
		event["execution"] = {
			"target": f"{args.gateway.rstrip('/')}{path}",
			"request": payload,
			"http_status": res.status_code,
			"ok": res.ok,
			"error": res.error,
		}
		log.append(event)
		print(f"Heard: {transcript!r} -> {parsed.intent} -> http_ok={res.ok} status={res.status_code}")


if __name__ == "__main__":
	main()
```

### 3.9 Where Rasa fits (optional NLU upgrade)

The MVP above is rule-based NLU.

To integrate Rasa, keep the same finite intents and swap Step 3.5 with:

- transcript → `rasa /model/parse` → intent + confidence (+ entities)

That Rasa integration is documented in:

- `SOLUTION_RASA_HF_VOICE_INTEG_LEROBOT_ROS2_MINIFI_CPP_PY_PART2.md`

---

## 4) MiNiFi impact (minimal, consistent with V1/V2)

MiNiFi changes are additive:

- Add a `TailFile` for `Logs/operator_commands.jsonl`.
- Reuse your existing parse/validate/rate-limit patterns.
- Route `event_type == operator_command` to:
	- storage / dashboards
	- alerting on repeated failures

The key boundary remains unchanged:

- MiNiFi observes/orchestrates.
- MiNiFi (and voice) do not become dependencies for the fixed-rate actuation loop.

---

## 5) Future work: frontend UI (web/desktop/mobile) that reuses the same contracts

You can treat “voice CLI” as the first operator client, and later add a GUI that speaks the exact same control-plane APIs.

### 4.1 Step-by-step UI roadmap (recommended)

#### Step 1 — Promote the HTTP gateway into an explicit “Operator Gateway API”

Keep the current localhost-only stance at first.

Add/standardize:

- request/response schemas for each endpoint
- consistent error codes (invalid transition, busy, not healthy, etc.)
- a `GET /v1/status` endpoint summarizing:
	- mission state
	- policy mode
	- last command + timestamp
	- health / loop jitter (from `controller_status`)

#### Step 2 — Provide a streaming telemetry channel for the UI

Two pragmatic MVP approaches:

1) **Tail JSONL and stream over WebSocket/SSE**
- a small “operator_gateway” process tails `Logs/*.jsonl`
- emits real-time updates to connected clients

2) **Subscribe to ROS2 topics and stream**
- gateway is a ROS2 node that subscribes and forwards

Start with (1) because it matches your MiNiFi strategy and doesn’t add DDS complexity to UI clients.

#### Step 3 — Build the client UI (React Native or alternatives)

Choose based on your target(s):

- Web: React (Next.js/Vite)
- Mobile: React Native (Expo) or Flutter
- Desktop: Tauri (Rust + web UI) or Electron

Recommended pragmatic path if you want “one codebase, many targets”:

- React Native + Expo for mobile, plus Expo Web for browser.

#### Step 4 — UI features (strictly operator-facing)

MVP screens/components:

- Current mode: Classical / Shadow / Mixed / Policy
- Mission state: idle/arming/takeoff/waypoint/landing/disarming
- Last voice/text command: transcript + parsed intent + success/failure
- Telemetry snapshot: NED position/velocity, armed/guided
- Safety controls: STOP / LAND / STOP MISSION button (always visible)

#### Step 5 — Integrate voice into the UI (optional)

- Mobile app can capture audio and run on-device ASR, or send audio to the local gateway for ASR.
- Keep the same intent vocabulary and confirmation rules.

### 4.2 Security and deployment notes (don’t skip later)

Once you move beyond localhost:

- require authentication (mTLS or token auth)
- add authorization (“who is allowed to switch to POLICY mode?”)
- rate limit operator commands
- keep an independent hardware kill-switch / failsafe path

