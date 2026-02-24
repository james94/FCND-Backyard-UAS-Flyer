
# Backyard Flyer — Proposed Repo Folder Structure (ROS2 Jazzy C++ → LeRobot + ROS2 Jazzy + MiNiFi C++)

This document proposes a folder structure that evolves the project from the existing **ROS2 Jazzy C++ + MiNiFi C++** approach into a **LeRobot-enabled ROS2** system (via `lerobot-ros`), while preserving the same safety boundary:

- **Actuation/control stays local** (C++ ROS2 control/vehicle nodes).
- **MiNiFi/NiFi are not in the actuation path** (telemetry + ops/control-plane only).
- **LeRobot integration is additive**: learning data + policy inference plug into the ROS2 system through stable topics.

It is intentionally consistent with:

- `SOLUTION_DESIGN_CPP.md` (fixed-rate loop, explicit concurrency)
- `SOLUTION_MODULAR_CPP.md` (core library boundary)
- `SOLUTION_ROS2_JAZZY_CPP.md` (ROS2 node wiring and messages)
- `SOLUTION_DESIGN_FOLDER_STRUCTURE_ROS2_MINIFI_CPP.md` (current ROS2+MiNiFi layout)
- `SOLUTION_MINIFI_CPP.md` and `SOLUTION_MINIFI_CPP_V2.md` (JSONL contracts + ops hooks)
- `README.md` (mission constraints)

---

## 1) What this structure must enable

1) Keep the deterministic controller logic as a plain C++ library.
2) Run ROS2 Jazzy nodes for control + transport + logging.
3) Add learning-facing ROS2 nodes (observations, action arbitration, recording).
4) Allow a LeRobot policy runner (often Python initially) without becoming a hard dependency for actuation.
5) Keep MiNiFi as a sidecar for telemetry + ops workflows.
6) Store and version datasets for imitation/RL training.

---

## 2) Staged evolution (recommended)

### Stage 0 — Current “ROS2 + MiNiFi” baseline

Matches `SOLUTION_DESIGN_FOLDER_STRUCTURE_ROS2_MINIFI_CPP.md`.

### Stage 1 — Add learning message contracts + observation/action plumbing

Add a new messages package and learning nodes (still no AI policy required).

### Stage 2 — Add dataset recording and offline training workspace

Create a place for:

- dataset artifacts (episodes)
- training configs
- evaluation scripts

### Stage 3 — Add policy inference node (LeRobot / lerobot-ros)

Introduce `lerobot_policy_runner_node` (likely Python at first). Keep `action_arbiter_node` as the safety gate.

### Stage 4 — Harden for production

- move inference to C++ (optional) using a compiled runtime (e.g., ONNX Runtime)
- add CI checks for message/schema compatibility
- add MiNiFi routing for episode/policy events

---

## 3) Recommended “final form” repo tree

```text
FCND-Backyard-UAS-Flyer/
	README.md

	# Core deterministic flight logic (no ROS2 headers)
	cpp/
		backyard_flyer_core/

	# ROS2 runtime
	ros2_ws/
		src/
			backyard_flyer_msgs/                 # existing: TelemetrySample, CommandIntent, ControllerStatus...
			backyard_flyer_core_ros/             # thin ament wrapper around cpp/backyard_flyer_core
			backyard_flyer_nodes/                # controller node + JSONL logger node
			backyard_flyer_vehicle/              # executes CommandIntent via bridge/MAVSDK
			backyard_flyer_http_gateway/         # localhost HTTP API for MiNiFi ops (optional, recommended)

			# LeRobot / learning integration (additive)
			backyard_flyer_learning_msgs/        # LearningObservation, LearningAction, PolicyMode, EpisodeEvent
			backyard_flyer_learning_nodes/       # learning_observation_node, action_arbiter_node, episode_recorder_node

			# Policy runner (often Python initially)
			backyard_flyer_lerobot_policy/       # a ROS2 package that runs the LeRobot policy runner node
																					 # (can be rclpy-based; kept separate from C++ nodes)

	# Runtime logs (gitignored)
	Logs/
		telemetry.jsonl
		controller_status.jsonl
		learning_events.jsonl

	# Learning datasets (gitignored or selectively tracked)
	data/
		lerobot/
			datasets/
				sim-01/
					2026-02-23T01-23-45Z/
						episodes/
							ep-000001.jsonl
							ep-000002.jsonl
						meta.json
			exports/                             # converted datasets, archives

	# AI workspace (source-controlled code/configs)
	ai/
		lerobot/
			README.md
			env/                                 # optional: conda/uv/poetry specs
			configs/
			scripts/
				train_imitation.py
				eval_policy.py
				export_onnx.py

	# Edge telemetry + ops
	edge/
		minifi-cpp/
			conf/
			scripts/

	# Platform ingestion
	platform/
		nifi/
			flows/
			docker/

	# Docs
	research/
		SOLUTION_DESIGN_CPP.md
		SOLUTION_MODULAR_CPP.md
		SOLUTION_ROS2_JAZZY_CPP.md
		SOLUTION_LEROBOT_ROS2_JAZZY_CPP.md
		SOLUTION_MINIFI_CPP.md
		SOLUTION_MINIFI_CPP_V2.md
		SOLUTION_DESIGN_FOLDER_STRUCTURE_CPP.md
		SOLUTION_DESIGN_FOLDER_STRUCTURE_ROS2_MINIFI_CPP.md
		SOLUTION_DESIGN_FOLDER_STRUCTURE_LEROBOT_ROS2_MINIFI_CPP.md

	.gitignore
```

---

## 4) How LeRobot, ROS2, and MiNiFi interact in this layout

### 4.1 Actuation/data plane (real-time-ish)

- `backyard_controller_node` produces classical intents and/or safety decisions.
- `action_arbiter_node` selects the authoritative `CommandIntent`.
- `vehicle_interface_node` executes that intent over the chosen transport.

### 4.2 Learning plane

- `learning_observation_node` publishes ML-friendly `LearningObservation`.
- `episode_recorder_node` records datasets.
- `backyard_flyer_lerobot_policy` consumes observations and publishes candidate actions.

Key boundary:

- Policy inference is **best-effort**; arbiter enforces timeouts and fallback.

### 4.3 Telemetry/ops plane (MiNiFi)

- `telemetry_jsonl_logger_node` writes JSONL logs under `Logs/`.
- MiNiFi tails those logs and ships them (buffering/backpressure/routing).
- MiNiFi triggers ops endpoints (`/v1/mission/start`, `/v1/policy/mode`, `/v1/episode/start`) using `InvokeHTTP`.

---

## 5) Notes on “C++ only” vs pragmatic hybrid

You can keep all safety-critical and real-time-ish nodes in C++.

For LeRobot, it’s common to start with:

- policy training + inference in Python

Then later evolve to:

- export the trained model to ONNX
- run inference in a C++ node (keeping the same `LearningAction` contract)

This is why the folder structure separates:

- `backyard_flyer_learning_nodes/` (C++ infra) from
- `backyard_flyer_lerobot_policy/` (policy runner package, likely Python initially)

The contracts stay stable either way.

