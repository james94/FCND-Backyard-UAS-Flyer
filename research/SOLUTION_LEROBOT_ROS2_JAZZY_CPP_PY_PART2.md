
# Backyard Flyer — LeRobot + ROS2 Jazzy (Part 2: Python Data/Training/Policy Pipeline + Modes)

This Part 2 document extends `SOLUTION_LEROBOT_ROS2_JAZZY_CPP.md` from “LeRobot-ready interfaces” into a **fully usable AI-driven autonomy pipeline**:

- record data from classical control (“Data only”)
- export to a **LeRobot-friendly dataset on disk**
- train an **imitation policy** to mimic the classical controller (“Offline imitation”)
- deploy a `lerobot_policy_runner_node` and validate it in **Shadow**, **Mixed**, and **Policy** modes

It is written as a follow-along guide. The code blocks are intentionally minimal and are meant to be typed into your own files.

Inputs re-analyzed:

- `SOLUTION_LEROBOT_ROS2_JAZZY_CPP.md` (Part 1: learning msgs + observation/arbiter patterns)
- `SOLUTION_ROS2_JAZZY_CPP.md` (baseline ROS2 controller/vehicle/logger design)
- `SOLUTION_DESIGN_CPP.md` and `SOLUTION_MODULAR_CPP.md` (fixed-rate loop + deterministic seams)
- `SOLUTION_MINIFI_CPP.md` and `SOLUTION_MINIFI_CPP_V2.md` (JSONL contracts + ops/control-plane boundary)
- `SOLUTION_DESIGN_FOLDER_STRUCTURE_LEROBOT_ROS2_MINIFI_CPP.md` and `SOLUTION_DESIGN_FOLDER_STRUCTURE_ROS2_MINIFI_CPP.md`
- `README.md` (mission constraints)

---

## 0) Does the “safe incremental path” exist already?

Yes: the Part 1 doc already proposes the safe staged rollout:

1) Data only
2) Offline imitation
3) Shadow mode
4) Mixed mode
5) Policy mode

What was missing is the concrete “how” for steps 2–5 (dataset format, training, deployment, evaluation, and how modes map to nodes/topics).

This Part 2 fills that in.

---

## 1) C++ vs Python decisions (made deliberately)

### Keep these in C++ (safety / determinism / real-time-ish)

- `vehicle_interface_node` (transport + actuation)
- `backyard_controller_node` (classical FSM + fixed-rate loop)
- `action_arbiter_node` (authoritative intent selection, timeouts, clamps, fallbacks)
- `learning_observation_node` (bounded observation building)
- `telemetry_jsonl_logger_node` (JSONL contracts for MiNiFi)
- `episode_recorder_node` (best-effort recording; isolate file I/O from control loop)

### Start these in Python (fast iteration / LeRobot ecosystem)

- dataset conversion/export to LeRobot-friendly storage
- offline training (imitation/RL)
- initial policy inference runner (`lerobot_policy_runner_node`) if LeRobot provides Python-first tooling
- evaluation tooling and plots

You can later migrate inference to C++ (e.g., ONNX Runtime) without changing the ROS2 contracts.

---

## 2) Data-only phase (concrete recording contract)

### 2.1 What you must record for imitation

For imitation learning you need, at minimum, time-aligned pairs:

- observation $o_t$
- action $a_t$ (what the classical controller did)

Optional but useful:

- episode boundary markers
- controller state (FSM state)
- any “termination reason” (landed, stopped, anomaly)

### 2.2 Define a *raw episode* file format (stable, simple)

Even if LeRobot’s preferred storage format evolves, a raw “source of truth” format helps you:

- re-export to new formats later
- debug recording issues with plain text

Recommended raw format: JSON Lines, one record per timestep:

`data/raw_runs/<vehicle_id>/<run_id>/episodes/<episode_id>.jsonl`

Each line:

```json
{
  "t_us": 1700000001123456,
  "episode_id": "ep-000001",
  "run_id": "2026-02-24T01-23-45Z",
  "vehicle_id": "sim-01",
  "observation": {
	"pos_n": 1.2,
	"pos_e": 0.3,
	"pos_d": -3.0,
	"vel_n": 0.1,
	"vel_e": 0.0,
	"vel_d": 0.0,
	"armed": true,
	"guided": true,
	"controller_state": 3,
	"in_mission": true
  },
  "action": {
	"type": 6,
	"north": 10.0,
	"east": 0.0,
	"down": -3.0,
	"heading_rad": 0.0,
	"takeoff_altitude_m": 0.0
  },
  "source": "classical"
}
```

Notes:

- `type` matches your `CommandIntent` enum values.
- This format is easy to generate from a ROS2 node and easy to parse from Python.

### 2.3 Episode recorder node (C++): what to subscribe to

To write the raw episode stream, your `episode_recorder_node` should subscribe to:

- `/learning/observation` (`LearningObservation`)
- `vehicle/command_intent` (authoritative `CommandIntent` after arbiter)
- `/learning/episode_event` (`EpisodeEvent`) (start/end)

Recording the *authoritative* intent ensures the dataset matches what was actually executed.

If you want “pure classical imitation” specifically:

- in `CLASSICAL` mode, authoritative == classical
- in later modes, you can still store both (executed action + policy-proposed action)

---

## 3) Export raw episodes to a LeRobot-friendly dataset on disk (Python)

### 3.1 Why a conversion step is the pragmatic MVP

LeRobot datasets are typically consumed through Python tooling (often built around Hugging Face `datasets` and Parquet/Arrow storage).

Instead of trying to perfectly match every LeRobot internal schema up front, do this:

1) record a stable raw JSONL format from ROS2
2) export into the dataset format your training code expects

This gives you forward compatibility.

### 3.2 A minimal exporter script (follow-along)

Create `ai/lerobot/scripts/export_raw_jsonl_to_dataset.py`:

```python
from __future__ import annotations

import argparse
import glob
import json
import os
from dataclasses import dataclass
from typing import Any, Dict, List


def read_jsonl(path: str) -> List[Dict[str, Any]]:
	out: List[Dict[str, Any]] = []
	with open(path, "r", encoding="utf-8") as f:
		for line in f:
			line = line.strip()
			if not line:
				continue
			out.append(json.loads(line))
	return out


def flatten_record(rec: Dict[str, Any]) -> Dict[str, Any]:
	obs = rec["observation"]
	act = rec["action"]
	return {
		"t_us": int(rec["t_us"]),
		"episode_id": rec["episode_id"],
		"run_id": rec["run_id"],
		"vehicle_id": rec["vehicle_id"],

		"pos_n": float(obs["pos_n"]),
		"pos_e": float(obs["pos_e"]),
		"pos_d": float(obs["pos_d"]),
		"vel_n": float(obs["vel_n"]),
		"vel_e": float(obs["vel_e"]),
		"vel_d": float(obs["vel_d"]),
		"armed": bool(obs["armed"]),
		"guided": bool(obs["guided"]),
		"controller_state": int(obs["controller_state"]),
		"in_mission": bool(obs["in_mission"]),

		"action_type": int(act["type"]),
		"action_n": float(act.get("north", 0.0)),
		"action_e": float(act.get("east", 0.0)),
		"action_d": float(act.get("down", 0.0)),
		"action_heading": float(act.get("heading_rad", 0.0)),
		"action_takeoff_alt": float(act.get("takeoff_altitude_m", 0.0)),
	}


def main() -> None:
	ap = argparse.ArgumentParser()
	ap.add_argument("--raw_root", required=True, help="data/raw_runs/<vehicle>/<run_id>")
	ap.add_argument("--out_dir", required=True, help="data/lerobot/datasets/<name>")
	args = ap.parse_args()

	episode_paths = sorted(glob.glob(os.path.join(args.raw_root, "episodes", "*.jsonl")))
	if not episode_paths:
		raise SystemExit(f"No episodes found under {args.raw_root}/episodes")

	rows: List[Dict[str, Any]] = []
	for ep in episode_paths:
		for rec in read_jsonl(ep):
			rows.append(flatten_record(rec))

	# MVP: save as JSONL (training scripts can ingest this).
	# If you use Hugging Face datasets, you can instead build a Dataset and save_to_disk().
	os.makedirs(args.out_dir, exist_ok=True)
	out_path = os.path.join(args.out_dir, "dataset.jsonl")
	with open(out_path, "w", encoding="utf-8") as f:
		for row in rows:
			f.write(json.dumps(row) + "\n")

	meta = {
		"raw_root": args.raw_root,
		"num_rows": len(rows),
		"schema": "backyard_flyer_minimal_v1",
	}
	with open(os.path.join(args.out_dir, "meta.json"), "w", encoding="utf-8") as f:
		json.dump(meta, f, indent=2)

	print(f"Wrote {len(rows)} rows to {out_path}")


if __name__ == "__main__":
	main()
```

How this relates to “LeRobot dataset format”:

- This produces a stable on-disk dataset directory that your LeRobot training code can ingest.
- If/when you want an exact LeRobot-native storage layout, evolve this exporter (it’s the only place you need to touch).

### 3.3 (Optional) Export as a Hugging Face `datasets` dataset on disk

Many robot-learning pipelines (including LeRobot-adjacent workflows) are happy when you provide a dataset in Hugging Face `datasets` format:

- Arrow/Parquet under the hood
- supports `load_from_disk()` and upload to the Hub

You can extend the exporter like this:

```python
from datasets import Dataset

dataset = Dataset.from_list(rows)
dataset.save_to_disk(args.out_dir)

# Optional: also produce Parquet for interoperability
dataset.to_parquet(os.path.join(args.out_dir, "dataset.parquet"))
```

If you do this, your training script can use `datasets.load_from_disk(args.out_dir)` instead of reading JSONL.

---

## 4) Offline imitation learning (Python)

Goal: train a policy $\pi(o_t) \approx a_t$ that mimics the classical controller.

### 4.1 Start with a constrained action space

For safety and learnability, imitate only one command type initially:

- `CMD_POSITION` targets (north/east/down/heading)
	- Here `down` is the **NED** coordinate used in your internal/ROS2 messages.
	- If the final actuator API expects altitude-up (e.g., UdaciDrone `cmd_position`), translate in the vehicle interface node/adapter.

Keep classical FSM in charge of:

- arming / takeoff / landing / disarming

This aligns naturally with **Mixed mode** later.

### 4.2 Minimal training script (sketch)

Create `ai/lerobot/scripts/train_imitation_cmd_position.py`:

```python
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from typing import List

import torch
from torch import nn


def load_jsonl(path: str) -> List[dict]:
	out = []
	with open(path, "r", encoding="utf-8") as f:
		for line in f:
			line = line.strip()
			if not line:
				continue
			out.append(json.loads(line))
	return out


class MlpPolicy(nn.Module):
	def __init__(self, in_dim: int, out_dim: int):
		super().__init__()
		self.net = nn.Sequential(
			nn.Linear(in_dim, 128), nn.ReLU(),
			nn.Linear(128, 128), nn.ReLU(),
			nn.Linear(128, out_dim),
		)

	def forward(self, x: torch.Tensor) -> torch.Tensor:
		return self.net(x)


def main() -> None:
	ap = argparse.ArgumentParser()
	ap.add_argument("--dataset_jsonl", required=True)
	ap.add_argument("--out_model", required=True, help="path to .pt")
	ap.add_argument("--epochs", type=int, default=10)
	args = ap.parse_args()

	rows = load_jsonl(args.dataset_jsonl)

	# Filter: only train on CMD_POSITION actions.
	CMD_POSITION = 6
	rows = [r for r in rows if int(r["action_type"]) == CMD_POSITION]
	if not rows:
		raise SystemExit("No CMD_POSITION rows found")

	# Observation vector (minimal): pos/vel + controller_state
	def obs_vec(r: dict) -> List[float]:
		return [
			r["pos_n"], r["pos_e"], r["pos_d"],
			r["vel_n"], r["vel_e"], r["vel_d"],
			float(r["controller_state"]),
		]

	def act_vec(r: dict) -> List[float]:
		return [r["action_n"], r["action_e"], r["action_d"], r["action_heading"]]

	x = torch.tensor([obs_vec(r) for r in rows], dtype=torch.float32)
	y = torch.tensor([act_vec(r) for r in rows], dtype=torch.float32)

	model = MlpPolicy(in_dim=x.shape[1], out_dim=y.shape[1])
	opt = torch.optim.Adam(model.parameters(), lr=1e-3)
	loss_fn = nn.MSELoss()

	model.train()
	for epoch in range(args.epochs):
		opt.zero_grad(set_to_none=True)
		pred = model(x)
		loss = loss_fn(pred, y)
		loss.backward()
		opt.step()
		print(f"epoch={epoch} loss={loss.item():.6f}")

	torch.save({"state_dict": model.state_dict()}, args.out_model)
	print(f"Saved {args.out_model}")


if __name__ == "__main__":
	main()
```

This is intentionally “not fancy”:

- It’s enough to validate the end-to-end pipeline.
- Once you trust the plumbing, you can replace this with LeRobot-native training utilities.

### 4.3 Export to a deployment format

For ROS2 policy runner deployment, you typically want a portable format.

Two common paths:

- Keep it in PyTorch (`.pt`) and run inference in Python.
- Export to ONNX and run inference in Python or C++.

---

## 5) Deploy a `lerobot_policy_runner_node` (Python, ROS2)

This node subscribes to `LearningObservation` and publishes `LearningAction`.

### 5.1 Policy runner contract

- Input topic: `/learning/observation`
- Output topic: `/learning/policy_action`
- Optional diagnostics topic: `/learning/policy_diag`

### 5.2 Policy runner skeleton (rclpy)

Create `ros2_ws/src/backyard_flyer_lerobot_policy/backyard_flyer_lerobot_policy/policy_runner_node.py`:

```python
from __future__ import annotations

import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from backyard_flyer_learning_msgs.msg import LearningObservation, LearningAction


class PolicyRunnerNode(Node):
	def __init__(self) -> None:
		super().__init__("lerobot_policy_runner")

		self.declare_parameter("mode_enabled", True)

		self.pub = self.create_publisher(LearningAction, "/learning/policy_action", 10)
		self.sub = self.create_subscription(
			LearningObservation,
			"/learning/observation",
			self.on_obs,
			10,
		)

		# TODO: load model here (PyTorch / ONNX) once you pick a format.

	def on_obs(self, msg: LearningObservation) -> None:
		if not self.get_parameter("mode_enabled").value:
			return

		t0 = time.perf_counter()

		# MVP: output a CMD_POSITION-like action.
		# Replace with real inference.
		out = LearningAction()
		out.type = 6  # CMD_POSITION
		out.north = float(msg.position_ned.north)
		out.east = float(msg.position_ned.east)
		out.down = float(msg.position_ned.down)
		out.heading_rad = 0.0
		out.takeoff_altitude_m = 0.0
		out.policy_confidence = 0.0
		out.t_us = msg.t_us
		out.run_id = msg.run_id

		self.pub.publish(out)

		dt_ms = (time.perf_counter() - t0) * 1000.0
		# Keep an eye on dt_ms; it must remain bounded.


def main() -> None:
	rclpy.init()
	node = PolicyRunnerNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
```

Production rule:

- policy runner is **best-effort**; it is allowed to drop frames.
- `action_arbiter_node` must enforce timeouts and fall back to classical intent.

---

## 6) Shadow/Mixed/Policy modes: exactly where they tie in

All modes are implemented by:

- `action_arbiter_node` (selection logic)
- an external “mode setter” (HTTP gateway endpoint, CLI, or ROS2 topic)

### 6.1 Add explicit modes beyond Part 1

In Part 1, `PolicyMode.msg` has `CLASSICAL`, `POLICY`, `MIXED`.

For validation workflows, it is helpful to add:

- `SHADOW` mode (policy runs, arbiter ignores policy for actuation)

If you want to keep messages stable, you can implement SHADOW as:

- `mode=CLASSICAL` for actuation
- plus a separate boolean `shadow_recording=true`

But simplest is to extend `PolicyMode.msg`:

```text
uint8 CLASSICAL=0
uint8 SHADOW=1
uint8 MIXED=2
uint8 POLICY=3
...
```

### 6.2 Arbiter logic by mode (recommended)

- **CLASSICAL**: publish classical intent.
- **SHADOW**: publish classical intent, but also log policy actions + diff metrics.
- **MIXED**: allow policy for a restricted subset (e.g., only `CMD_POSITION`). Classical always wins for `STOP`, `LAND`, and state transitions.
- **POLICY**: use policy actions broadly, but keep safety overrides + clamps + timeouts.

### 6.3 Mixed-mode gating (concrete rule)

Recommended first mixed-mode rule:

- policy can only propose `CMD_POSITION`
- all other types (ARM/DISARM/TAKEOFF/LAND/STOP/TAKE_CONTROL/RELEASE_CONTROL) come from classical

Why:

- it matches the mission’s structure
- it reduces the chance that policy breaks safety-critical sequencing

---

## 7) How to validate policy vs classical (evaluation pipeline)

Validation must be measurable, not vibes.

### 7.1 Add a `policy_evaluator_node` (Python)

In SHADOW mode, the evaluator compares:

- classical/authoritative intent
- policy-proposed action

and publishes a small metric message or writes a JSONL report.

Minimal JSONL line:

```json
{
  "event_type": "policy_eval",
  "t_us": 1700000002123456,
  "run_id": "...",
  "vehicle_id": "...",
  "mode": "SHADOW",
  "action_type": 6,
  "err_n": 0.42,
  "err_e": 0.10,
  "err_d": 0.00,
  "err_heading": 0.05
}
```

Write this under `Logs/policy_eval.jsonl` so MiNiFi can ship it.

### 7.2 What “good enough” looks like

For MVP, define simple acceptance criteria:

- in SHADOW mode, policy error percentiles stay below thresholds
- in MIXED mode, mission success rate stays high
- in POLICY mode, arbiter fallbacks are rare (policy is timely) and safety overrides are rare

---

## 8) MiNiFi interaction in Part 2 (what changes)

Mechanics do not change:

- ROS2 writes JSONL under `Logs/`
- MiNiFi tails and forwards
- MiNiFi triggers ops via HTTP gateway (`InvokeHTTP`)

### 8.1 New JSONL logs to tail

- `Logs/learning_events.jsonl` (episode boundaries, mode changes)
- `Logs/policy_eval.jsonl` (shadow diffs)
- optionally `Logs/policy_actions.jsonl` (policy outputs, rate-limited)

### 8.2 Routing ideas (still optional)

- route `policy_eval` to alerting if error spikes
- route `episode_boundary` to dataset tracking storage

---

## 9) Where LLM/LVM fits (without breaking real-time)

Keep LLM/LVM out of the actuation loop.

Good fits that preserve determinism:

- convert operator intent (NL) → high-level mission spec (box size, altitude, pattern)
- generate *candidate* waypoint plans that the classical planner validates
- produce post-run summaries, anomaly explanations, dataset labels

How to integrate safely:

- run LLM/LVM as a separate process/service
- communicate via the ops/control-plane (HTTP gateway) using validated, versioned schemas
- apply changes only at safe boundaries (mission start, between episodes)

---

## 10) End-to-end checklist (what you should be able to do)

1) Run classical ROS2 mission and record raw episodes.
2) Export raw episodes to a training dataset directory.
3) Train an imitation policy.
4) Start policy runner and run SHADOW mode; inspect `policy_eval.jsonl`.
5) Enable MIXED mode for `CMD_POSITION` only.
6) Graduate to POLICY mode with strict timeout + clamps + safety overrides.

### 10.1 Minimal run order (operationally)

1) Start ROS2 “classical” stack (controller, vehicle, logger).
2) Start learning plumbing (observation, arbiter, recorder).
3) Fly a run in `CLASSICAL` mode and confirm raw episodes exist under `data/raw_runs/...`.
4) Run the exporter to produce `data/lerobot/datasets/...`.
5) Train imitation policy and save a model artifact.
6) Start `lerobot_policy_runner_node`.
7) Switch to **SHADOW** mode and confirm:
	- `policy_eval.jsonl` is being written
	- MiNiFi can ship it (if enabled)
8) Switch to **MIXED** mode (policy only for `CMD_POSITION`).
9) Switch to **POLICY** mode only after:
	- timeouts are correct
	- safety clamps are correct
	- mission success in MIXED is stable

