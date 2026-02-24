
# Backyard Flyer — Folder Structure (React Native UI + Operator Gateway + Voice, Full Stack)

This folder-structure guide extends your existing layout to include a **React Native (Expo) frontend** that can target:

- smartphone (iOS/Android)
- web (Expo web)
- desktop (wrap the web build in Tauri/Electron, or evolve later)

…while integrating cleanly with:

- ROS2 Jazzy C++ autonomy stack
- LeRobot policy pipeline (Shadow/Mixed/Policy)
- MiNiFi C++ telemetry/ops sidecar
- Operator voice stack (HF ASR + Rasa + action server)

Core invariants (unchanged):

- UI does **not** talk ROS2/DDS.
- UI uses a **network-facing Operator Gateway** (REST + WS) that proxies safe `/v1/*` commands to the **localhost** `ros2_http_gateway_node`.
- UI consumes telemetry as **structured events**, typically by subscribing to the gateway WebSocket.

---

## 1) Recommended repo tree additions

This is a “final form” tree showing where UI/gateway fit next to your current operator voice/Rasa pieces.

```text
FCND-Backyard-UAS-Flyer/
	README.md

	# ROS2 runtime (existing)
	ros2_ws/
		src/
			backyard_flyer_msgs/
			backyard_flyer_nodes/
			backyard_flyer_vehicle/
			backyard_flyer_http_gateway/         # localhost /v1/* control-plane

			backyard_flyer_learning_msgs/
			backyard_flyer_learning_nodes/
			backyard_flyer_lerobot_policy/
			backyard_flyer_policy_eval/

	# Runtime logs (existing; MiNiFi tails these)
	Logs/                                    # gitignored
		telemetry.jsonl
		controller_status.jsonl
		learning_events.jsonl
		policy_eval.jsonl
		operator_commands.jsonl                # voice + UI audit stream
		rasa_dialogue.jsonl                    # optional

	# Operator stack (extended)
	operator/
		voice/                                 # HF ASR + (optional) Rasa client
			app/
			requirements.txt

		rasa/                                  # NLU + dialogue + action server
			config.yml
			domain.yml
			data/
			actions/

		gateway/                               # NEW: network-facing API for UIs
			README.md
			app/
				server.py                          # REST + WS; proxies to ros2_http_gateway_node
				tailer.py                           # tails Logs/*.jsonl -> structured events
				auth.py                             # optional API key / simple auth
				types.py                            # event envelope definitions
			requirements.txt

		ui/                                    # NEW: frontend apps
			mobile/                              # React Native (Expo) single codebase
				app.json
				package.json
				tsconfig.json
				src/
					api/
					components/
					screens/
					state/
				App.tsx

			desktop/                             # optional wrapper around the web build
				README.md
				# Tauri/Electron config lives here (future)

	# Edge sidecar (existing)
	edge/
		minifi-cpp/
			conf/
				config.yml                         # tails Logs/*.jsonl

	# Data + models (existing)
	data/
		raw_runs/
		lerobot/
			datasets/
			models/

	# Docs
	research/
		SOLN_UAS_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK.md
		SOLN_DES_DIR_STRUCTURE_REACT_INTEG_AI_AUTONOMY_VOICE_FULL_STACK.md
```

---

## 2) Process boundaries (important)

Recommended processes (minimum):

- ROS2 processes (C++ + Python policy runner)
	- `ros2_http_gateway_node` (localhost only)
	- controller/vehicle/logger/learning/arbiter nodes

- Operator-facing processes
	- `operator/gateway` (network-facing): exposes REST + WS to UI clients
	- optional `operator/voice` and `operator/rasa` (voice/NLU)
	- MiNiFi edge sidecar (tails JSONL)

UI clients (phone/web/desktop):

- talk only to `operator/gateway`
- never depend on ROS2/DDS

---

## 3) Why the gateway is a separate directory (and not inside ROS2)

You *could* add streaming endpoints to `ros2_http_gateway_node`, but keeping a separate operator gateway is pragmatic:

- it can tail JSONL logs without complicating the ROS2 node
- it can handle WebSocket fanout and auth without disturbing real-time-ish code
- it becomes the stable interface for multiple clients (web/mobile/desktop)

The ROS2 gateway stays minimal and local:

- `POST /v1/*` control-plane only
- request handlers must remain non-blocking (handoff to safe boundary)

---

## 4) Minimal configuration notes

### 4.1 UI runtime configuration

The Expo app should point at the operator gateway base URL via environment variable:

- `EXPO_PUBLIC_API_BASE_URL=http://<gateway-host>:8088`

### 4.2 Logging + correlation

To make the UI “workflow view” correct:

- ensure all event streams include `vehicle_id`, `run_id`, and `t_us`
- the gateway should filter/stream per `(vehicle_id, run_id)` when requested

---

## 5) What NOT to do

- Don’t make the UI call the ROS2 localhost gateway directly from a phone.
- Don’t stream raw JSONL files to the UI.
- Don’t route actuation through MiNiFi or through the UI.

