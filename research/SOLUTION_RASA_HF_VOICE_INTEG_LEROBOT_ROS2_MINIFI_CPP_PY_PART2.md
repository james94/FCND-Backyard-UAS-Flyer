
# Backyard Flyer — Part 2: Rasa + Hugging Face Voice Commands (LeRobot + ROS2 Jazzy + MiNiFi)

This Part 2 document adds a **concrete implementation path** for integrating:

- Hugging Face ASR (voice → text)
- Rasa (text → intent + dialogue/confirmation)

…while preserving your existing end-to-end autonomy architecture:

- **Actuation plane (C++ / deterministic):** `action_arbiter_node` → `vehicle/command_intent` → `vehicle_interface_node`
- **Control plane (ops):** `ros2_http_gateway_node` localhost endpoints (start/stop/mode/episode/safety)
- **Telemetry plane:** JSONL logs tailed by MiNiFi

Inputs re-analyzed for consistency:

- `SOLUTION_HF_VOICE_INTEG_LEROBOT_ROS2_MINIFI_CPP_PY.md` (HF voice control-plane approach)
- `SOLN_DES_DIR_STRUCTURE_HF_VOICE_INTEG_LEROBOT_ROS2_MINIFI_CPP_PY.md` (HF voice folder layout)
- `SOLUTION_LEROBOT_ROS2_JAZZY_CPP.md` and `SOLUTION_LEROBOT_ROS2_JAZZY_CPP_PY_PART2.md` (modes, datasets, policy runner)
- `SOLUTION_ROS2_JAZZY_CPP.md` (HTTP gateway as MiNiFi-friendly control-plane seam)
- `SOLUTION_MINIFI_CPP.md` and `SOLUTION_MINIFI_CPP_V2.md` (JSONL + HTTP ops boundary)
- `README.md` (mission semantics)

---

## 0) Why Rasa is a good fit here

Rasa helps with the two hard parts of operator voice control:

1) **NLU (intent classification)** into a finite set of allowed commands.
2) **Dialogue/confirmations** (e.g., “I heard policy mode; confirm?”) in a structured way.

You still keep the same critical design choice:

- Rasa can decide *which* high-level command the operator wants.
- The ROS2 system + arbiter still decide *how* to act safely.

---

## 1) Architecture: HF ASR + Rasa + ROS2 gateway

MVP recommended dataflow:

```
Mic -> HF ASR (Python) -> transcript -> Rasa parse -> (intent, entities)
																						|
																						v
																 Rasa action server executes:
																		HTTP POST to ros2_http_gateway_node

All attempts -> Logs/operator_commands.jsonl
```

Why run the HTTP calls in the Rasa action server:

- Rasa “actions” are already the standard place for side effects.
- You get dialogue state/slots for confirmations.

Alternative MVP (also valid):

- HF voice service calls `POST /model/parse` and executes HTTP itself.
- Rasa is “intent engine only”.

This doc shows the action-server pattern because it scales best.

---

## 2) Step-by-step: create a Rasa project for your finite intent vocabulary

### Step 1 — Install Rasa

Create `operator/rasa/requirements.txt` (example):

```text
rasa>=3.6
requests>=2.31
```

Notes:

- Rasa packaging can be sensitive to Python versions; keep it pinned per your environment.

### Step 2 — Initialize a project

From `operator/rasa/`:

```bash
rasa init --no-prompt
```

You will replace the example intents/stories with your own.

### Step 3 — Define intents (only the allowed set)

In `domain.yml`, keep intents finite:

```yaml
intents:
	- mission_start_classical
	- mission_stop
	- policy_mode_shadow
	- policy_mode_mixed_cmd_position
	- policy_mode_policy_cmd_position
	- episode_start
	- episode_stop
	- emergency_stop
```

Important:

- Do not add “free-form navigation” intents that would allow arbitrary actuation.

### Step 4 — Add NLU examples (`data/nlu.yml`)

```yaml
version: "3.1"

nlu:
	- intent: mission_start_classical
		examples: |
			- start mission classical
			- start the classical mission
			- run the classical controller

	- intent: policy_mode_shadow
		examples: |
			- enable shadow mode
			- shadow mode
			- run policy in shadow mode

	- intent: policy_mode_mixed_cmd_position
		examples: |
			- enable mixed mode position
			- mixed mode cmd position
			- mixed mode for position only

	- intent: policy_mode_policy_cmd_position
		examples: |
			- enable policy mode position
			- policy mode cmd position
			- policy mode for position only

	- intent: mission_stop
		examples: |
			- stop mission
			- end mission
			- abort mission

	- intent: emergency_stop
		examples: |
			- emergency stop
			- stop now
			- e stop
			- kill switch
```

### Step 5 — Add rules for confirmations (Mixed/Policy)

Switching to Mixed/Policy should require confirmation.

In `data/rules.yml`:

```yaml
version: "3.1"

rules:
	- rule: Ask confirmation before mixed mode
		steps:
			- intent: policy_mode_mixed_cmd_position
			- action: utter_confirm_mixed
			- action: action_set_pending_command

	- rule: Ask confirmation before policy mode
		steps:
			- intent: policy_mode_policy_cmd_position
			- action: utter_confirm_policy
			- action: action_set_pending_command

	- rule: Confirm yes
		steps:
			- intent: affirm
			- action: action_execute_pending_command

	- rule: Confirm no
		steps:
			- intent: deny
			- action: action_clear_pending_command
			- action: utter_cancelled
```

And in `domain.yml` add the utterances:

```yaml
responses:
	utter_confirm_mixed:
		- text: "Confirm enabling MIXED mode (CMD_POSITION only). Say yes to proceed."
	utter_confirm_policy:
		- text: "Confirm enabling POLICY mode (CMD_POSITION only). Say yes to proceed."
	utter_cancelled:
		- text: "Cancelled."
```

### Step 6 — Create an action server that calls the ROS2 HTTP gateway

In `actions/actions.py`:

```python
from __future__ import annotations

import json
import os
import time
import uuid
from typing import Any, Dict, List, Text

import requests
from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import SlotSet


CMD_POSITION = 6


def now_us() -> int:
		return int(time.time() * 1_000_000)


def append_jsonl(path: str, obj: Dict[str, Any]) -> None:
		os.makedirs(os.path.dirname(path), exist_ok=True)
		with open(path, "a", encoding="utf-8") as f:
				f.write(json.dumps(obj) + "\n")


class ActionSetPendingCommand(Action):
		def name(self) -> Text:
				return "action_set_pending_command"

		def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Any]:
				intent = (tracker.latest_message.get("intent") or {}).get("name")
				return [SlotSet("pending_command", intent)]


class ActionClearPendingCommand(Action):
		def name(self) -> Text:
				return "action_clear_pending_command"

		def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Any]:
				return [SlotSet("pending_command", None)]


class ActionExecutePendingCommand(Action):
		def name(self) -> Text:
				return "action_execute_pending_command"

		def run(self, dispatcher: CollectingDispatcher, tracker: Tracker, domain: Dict[Text, Any]) -> List[Any]:
				gateway = os.environ.get("ROS2_GATEWAY", "http://127.0.0.1:8080").rstrip("/")
				vehicle_id = os.environ.get("VEHICLE_ID", "sim-01")
				run_id = os.environ.get("RUN_ID", f"rasa-{uuid.uuid4().hex[:8]}")

				pending = tracker.get_slot("pending_command")
				transcript = (tracker.latest_message.get("text") or "").strip()

				if not pending:
						dispatcher.utter_message(text="No pending command.")
						return []

				path = None
				payload: Dict[str, Any] = {}
				if pending == "policy_mode_mixed_cmd_position":
						path = "/v1/policy/mode"
						payload = {"mode": "mixed", "allow_action_types": [CMD_POSITION]}
				elif pending == "policy_mode_policy_cmd_position":
						path = "/v1/policy/mode"
						payload = {"mode": "policy", "allow_action_types": [CMD_POSITION]}
				else:
						dispatcher.utter_message(text=f"Unsupported pending command: {pending}")
						return [SlotSet("pending_command", None)]

				url = f"{gateway}{path}"
				ok = False
				status = 0
				err = None
				try:
						r = requests.post(url, json=payload, timeout=1.0)
						status = r.status_code
						ok = 200 <= status < 300
						if not ok:
								err = r.text[:500]
				except Exception as e:
						err = str(e)

				append_jsonl(
						"Logs/operator_commands.jsonl",
						{
								"event_type": "operator_command",
								"t_us": now_us(),
								"vehicle_id": vehicle_id,
								"run_id": run_id,
								"input": {"transcript": transcript, "source": "rasa"},
								"decision": {"intent": pending, "confidence": None, "requires_confirm": True},
								"execution": {"target": url, "request": payload, "http_status": status, "ok": ok, "error": err},
						},
				)

				dispatcher.utter_message(text=("OK" if ok else f"Failed ({status}): {err}"))
				return [SlotSet("pending_command", None)]
```

In `domain.yml` add slots/actions:

```yaml
slots:
	pending_command:
		type: text
		influence_conversation: true

actions:
	- action_set_pending_command
	- action_execute_pending_command
	- action_clear_pending_command
```

### Step 7 — Non-confirmation commands can execute immediately

For commands like `mission_stop` and `emergency_stop`, you can create direct actions (no confirmation):

- `ActionMissionStop` → `POST /v1/mission/stop`
- `ActionEmergencyStop` → `POST /v1/safety/stop`

This preserves a critical safety property:

- “stop now” should not require a multi-turn confirmation.

---

## 3) Step-by-step: connect Hugging Face ASR to Rasa

Rasa expects text input.

So the simplest integration is:

1) HF ASR transcribes audio to `transcript`
2) send transcript to Rasa REST channel

Example (pseudo-code) from your voice service:

```python
import requests

def send_to_rasa(transcript: str, sender: str = "operator") -> None:
		requests.post(
				"http://127.0.0.1:5005/webhooks/rest/webhook",
				json={"sender": sender, "message": transcript},
				timeout=2.0,
		)
```

If you only want intent classification (no dialogue):

```python
r = requests.post(
		"http://127.0.0.1:5005/model/parse",
		json={"text": transcript},
		timeout=2.0,
)
intent = r.json()["intent"]["name"]
confidence = float(r.json()["intent"]["confidence"])
```

---

## 4) How this interacts with your autonomy pipeline (modes + safety)

This preserves the same rollout strategy from `SOLUTION_LEROBOT_ROS2_JAZZY_CPP_PY_PART2.md`:

- Voice/Rasa can request mode changes.
- `action_arbiter_node` enforces:
	- strict staleness timeouts for policy actions
	- clamps and safety overrides
	- allow-listing of action types (e.g., only `CMD_POSITION`)

This is the intended safety boundary:

- Even if Rasa misclassifies, the backend should reject invalid transitions or clamp unsafe actions.

---

## 5) Future work: frontend UI (React Native/web/desktop)

Same as in the HF-only doc:

- keep the control-plane API stable (`/v1/*`)
- stream telemetry and operator command events to the UI (WebSocket/SSE)
- show: mission state, policy mode, last operator command, and safety STOP controls

You can keep Rasa as a backend “dialogue service” and have the UI act as the voice + display client.

