
# Backyard Flyer — Folder Structure (HF Voice Commands + LeRobot + ROS2 Jazzy + MiNiFi)

This folder-structure addendum complements:

- `SOLUTION_DESIGN_FOLDER_STRUCTURE_ROS2_MINIFI_CPP.md` (ROS2 + MiNiFi baseline)
- `SOLUTION_DESIGN_FOLDER_STRUCTURE_LEROBOT_ROS2_MINIFI_CPP.md` (LeRobot-enabled ROS2 additions)
- `SOLN_DES_DIR_STRUCTURE_LEROBOT_ROS2_MINIFI_CPP_PY_PART2.md` (Part 2: training + policy runner)
- `SOLUTION_HF_VOICE_INTEG_LEROBOT_ROS2_MINIFI_CPP_PY.md` (this feature’s architecture)

Goal:

- Add a **Python voice command service** (Hugging Face ASR + intent parsing) that calls your existing **localhost control-plane** (ROS2 HTTP gateway).
- Add an **operator command JSONL stream** so MiNiFi/NiFi and future UIs can observe what the operator said, what intent was parsed, and what was executed.
- Leave the existing safety boundary unchanged.

---

## 1) Minimal additions (MVP)

This is the smallest set of repo additions that still provides a clean, production-friendly boundary.

```text
FCND-Backyard-UAS-Flyer/
	README.md

	# ROS2 runtime (existing)
	ros2_ws/
		src/
			backyard_flyer_msgs/
			backyard_flyer_nodes/
			backyard_flyer_vehicle/
			backyard_flyer_http_gateway/          # exposes localhost /v1/* endpoints

			backyard_flyer_learning_msgs/
			backyard_flyer_learning_nodes/
			backyard_flyer_lerobot_policy/        # rclpy policy runner (Part 2)
			backyard_flyer_policy_eval/           # rclpy policy evaluator (Part 2)

	# Operator tooling (new)
	operator/
		voice/
			README.md
			requirements.txt
			app/
				voice_service.py                    # mic -> ASR -> intent -> HTTP call
				intent_parser.py                    # transcript -> finite intent vocabulary
				http_client.py                      # calls http://127.0.0.1:<port>/v1/*
				jsonl_logger.py                     # writes Logs/operator_commands.jsonl
			scripts/
				run_voice.sh

	# Runtime logs (MiNiFi tails these; all are gitignored)
	Logs/
		telemetry.jsonl
		controller_status.jsonl
		learning_events.jsonl
		policy_eval.jsonl
		operator_commands.jsonl                 # NEW

	# Data and models (existing Part 2 structure)
	data/
		raw_runs/
		lerobot/
			datasets/
			models/

	# Edge sidecar (existing)
	edge/
		minifi-cpp/
			conf/
				config.yml                          # add TailFile for operator_commands.jsonl

	research/
		SOLUTION_HF_VOICE_INTEG_LEROBOT_ROS2_MINIFI_CPP_PY.md
		SOLN_DES_DIR_STRUCTURE_HF_VOICE_INTEG_LEROBOT_ROS2_MINIFI_CPP_PY.md
```

Notes:

- The voice service is intentionally **not** placed inside `ros2_ws/src/` for the MVP because it does not need DDS/ROS2 to do its job; it talks to the control-plane via HTTP.
- If you later decide you want voice to be a ROS2 node, you can move it under `ros2_ws/src/backyard_flyer_voice_cmd/` as an `ament_python` package (see Section 3).

---

## 2) Log/contract additions (for MiNiFi + observability)

### 2.1 New log stream

- `Logs/operator_commands.jsonl`

Event types (recommended):

- `operator_command` (one record per voice command attempt)

This mirrors the existing log streams:

- `telemetry.jsonl`
- `controller_status.jsonl`
- `learning_events.jsonl`
- `policy_eval.jsonl`

### 2.2 MiNiFi config impact

In `edge/minifi-cpp/conf/config.yml` (and any V2 flows), add:

- `TailFile` for `Logs/operator_commands.jsonl`
- parsing/validation processor (can be a lightweight JSON validator)
- route failures (e.g., repeated ASR/NLU failures) to alerts

No other MiNiFi architecture needs to change.

---

## 3) Optional: make voice a ROS2 package (if you want tighter runtime supervision)

If you prefer voice to live inside ROS2 (so `ros2 launch` brings it up), add:

```text
ros2_ws/
	src/
		backyard_flyer_voice_cmd/               # ament_python package
			package.xml
			setup.py
			backyard_flyer_voice_cmd/
				__init__.py
				voice_node.py                       # still uses HF ASR + intent parser
				http_client.py                      # still calls /v1/* endpoints (or ROS2 services)
```

Why you might do this later:

- easier launch orchestration
- easier health checks
- consistent logging/parameters

Why keeping HTTP is still valuable:

- UI clients (web/mobile/desktop) can reuse the same gateway without DDS.

---

## 4) Future work placement: frontend apps and an operator gateway

When you add a real UI, it’s helpful to add a dedicated `operator_gateway/` component that:

- exposes REST for commands (reusing `/v1/*` semantics)
- exposes WebSocket/SSE for live telemetry and status
- can tail JSONL logs or subscribe to ROS2 topics

Suggested placement (future):

```text
FCND-Backyard-UAS-Flyer/
	operator/
		gateway/
			README.md
			app/
				server.py                           # REST + WS; bridges to ros2_http_gateway_node
		ui/
			mobile/                               # React Native (Expo)
			web/                                  # React web
			desktop/                              # Tauri/Electron wrapper (optional)
```

This keeps a clean layering:

- UI clients never need direct ROS2/DDS.
- The safety boundary remains: arbiter + vehicle node are authoritative.

