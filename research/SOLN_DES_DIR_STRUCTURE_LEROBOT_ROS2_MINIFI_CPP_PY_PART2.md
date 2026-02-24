
# Backyard Flyer — Folder Structure (LeRobot + ROS2 Jazzy + MiNiFi, Part 2: Python Training + Deployment)

This Part 2 folder structure complements:

- `SOLUTION_DESIGN_FOLDER_STRUCTURE_LEROBOT_ROS2_MINIFI_CPP.md` (Part 1: C++/ROS2 learning infra)
- `SOLUTION_LEROBOT_ROS2_JAZZY_CPP_PY_PART2.md` (Part 2: export/train/deploy/evaluate)

The goal is to add:

- raw episode recording storage
- dataset export and offline training workspace
- a deployable policy runner package (Python-first)
- evaluation artifacts and logs that MiNiFi can ship

without changing the safety boundary:

- actuation loop + arbiter remain in C++
- MiNiFi remains sidecar telemetry/ops

---

## 1) Recommended repo additions (final form)

```text
FCND-Backyard-UAS-Flyer/
	README.md

	# Core deterministic flight logic
	cpp/
		backyard_flyer_core/

	# ROS2 runtime (C++ safety-critical + plumbing)
	ros2_ws/
		src/
			backyard_flyer_msgs/
			backyard_flyer_core_ros/
			backyard_flyer_nodes/
			backyard_flyer_vehicle/
			backyard_flyer_http_gateway/

			# Learning contracts + nodes (C++)
			backyard_flyer_learning_msgs/
			backyard_flyer_learning_nodes/

			# Policy runner + evaluation tools (Python)
			backyard_flyer_lerobot_policy/       # rclpy node: lerobot_policy_runner_node
			backyard_flyer_policy_eval/          # rclpy node: policy_evaluator_node (shadow diffs)

	# Runtime logs written by ROS2 nodes (MiNiFi tails these)
	Logs/                                    # gitignored
		telemetry.jsonl
		controller_status.jsonl
		learning_events.jsonl
		policy_eval.jsonl

	# Raw recorded episodes (source of truth)
	data/                                    # usually gitignored; optionally track small samples
		raw_runs/
			<vehicle_id>/
				<run_id>/
					episodes/
						ep-000001.jsonl
						ep-000002.jsonl
					meta.json

		# Exported datasets for training
		lerobot/
			datasets/
				backyard_flyer_minimal_v1/
					dataset.jsonl
					meta.json
			models/
				imitation_cmd_position/
					policy.pt
					policy.onnx

	# AI / LeRobot workspace (Python training/export scripts)
	ai/
		lerobot/
			README.md
			env/
				requirements.txt                   # or uv/poetry/conda spec
			scripts/
				export_raw_jsonl_to_dataset.py
				train_imitation_cmd_position.py
				export_onnx.py

	# Edge sidecar
	edge/
		minifi-cpp/
			conf/
				config.yml                         # tail Logs/*.jsonl and forward
			scripts/
				run_minifi.sh

	# Platform
	platform/
		nifi/

	research/
		SOLUTION_LEROBOT_ROS2_JAZZY_CPP.md
		SOLUTION_LEROBOT_ROS2_JAZZY_CPP_PY_PART2.md
		SOLN_DES_DIR_STRUCTURE_LEROBOT_ROS2_MINIFI_CPP_PY_PART2.md

	.gitignore
```

---

## 2) Why raw episodes and exported datasets are separate

Keep two layers:

1) `data/raw_runs/...` (recorded from ROS2, stable JSONL source-of-truth)
2) `data/lerobot/datasets/...` (exported training-ready representation)

Benefits:

- you can change training schema without touching ROS2 nodes
- you can re-export older runs when your model/features evolve
- debugging is easier (raw JSONL is readable)

---

## 3) Where Shadow/Mixed/Policy mode artifacts live

### Shadow mode

- `Logs/policy_eval.jsonl` contains action diffs
- optionally `Logs/policy_actions.jsonl` contains policy outputs (rate-limited)

### Mixed/Policy modes

- `Logs/learning_events.jsonl` records mode changes + episode boundaries
- `Logs/controller_status.jsonl` continues to record loop health/jitter

MiNiFi should tail and ship these like any other JSONL stream.

---

## 4) MiNiFi config impact (minimal)

In `edge/minifi-cpp/conf/config.yml`:

- add additional `TailFile` processors for `Logs/learning_events.jsonl` and `Logs/policy_eval.jsonl`
- reuse the same parse/validate/rate-limit patterns as in `SOLUTION_MINIFI_CPP.md`

The key boundary is unchanged:

- MiNiFi observes and orchestrates
- MiNiFi never becomes a dependency for actuation

