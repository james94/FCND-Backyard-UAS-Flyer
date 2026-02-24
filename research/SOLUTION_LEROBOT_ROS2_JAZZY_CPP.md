
# Backyard Flyer — LeRobot + ROS2 Jazzy C++ Production Approach (Custom Nodes, Follow-Along Guide)

This document extends the existing ROS2 Jazzy C++ approach in `SOLUTION_ROS2_JAZZY_CPP.md` into a **LeRobot-enabled ROS2 system** (via `lerobot-ros`) so you can:

- keep your **real-time-ish actuation loop** and safety boundary in **C++ / ROS2 Jazzy**, and
- add a clean path to **data collection**, **offline training**, and eventually **AI-driven autonomy** (DL / RL / LLM/LVM-assisted behaviors) when you’re ready.

It is written as a “type-it-yourself” follow-along: you can copy the code blocks into your own files, validate incrementally, and learn the architecture.

Inputs re-analyzed:

- `SOLUTION_ROS2_JAZZY_CPP.md` (current ROS2 node split + command-intent seam)
- `SOLUTION_DESIGN_FOLDER_STRUCTURE_ROS2_MINIFI_CPP.md` (ROS2 + MiNiFi repo layout)
- `SOLUTION_MINIFI_CPP.md` and `SOLUTION_MINIFI_CPP_V2.md` (JSONL telemetry + ops/control-plane boundary)
- `SOLUTION_DESIGN_CPP.md` and `SOLUTION_MODULAR_CPP.md` (fixed-rate loop model + `IVehicle` seam)
- `SOLUTION_DESIGN_FOLDER_STRUCTURE_CPP.md` (staged evolution)
- `README.md` (mission requirements and event-driven state machine constraints)

Mission reminder:

- Fly a **10m box** at **3m altitude**, then land/disarm/end mission.
- Local frame is **NED**; 3m altitude corresponds to `down = -3.0`.

---

## 0) What changes when you introduce LeRobot / robot learning

### 0.1 Keep the same safety boundary

The most important architectural rule from the MiNiFi docs still applies:

- **Actuation** stays local and deterministic (your C++ controller loop + vehicle interface).
- **Data/ops workflows** (shipping logs, starting/stopping runs, orchestration) are allowed to be “eventual” and resilient (MiNiFi/NiFi).

LeRobot adds two new “planes”:

1) **Learning data plane** (record observations/actions/rewards/episode boundaries)
2) **Policy plane** (inference node producing actions from observations)

You do *not* want your actuation loop to become dependent on:

- Python GC / dynamic allocations
- a heavy ML runtime
- network calls to model servers

So: integrate policy output through a **bounded, time-limited interface** with **fallback**.

### 0.2 The key seam: “Command Intent” stays the only actuation contract

From `SOLUTION_ROS2_JAZZY_CPP.md`, the clean ROS2 split is:

- controller node makes decisions → publishes `CommandIntent`
- vehicle node executes `CommandIntent`

For LeRobot, keep that same seam.

Learned autonomy should output either:

- a `CommandIntent` (same message), or
- a *higher-level* intent that your safety layer translates into a `CommandIntent`.

This allows a smooth transition from classical FSM control → learned policy control.

---

## 1) Target end-state (LeRobot + ROS2 + MiNiFi)

We extend the ROS2 node set with **learning-facing nodes** while preserving the real-time-ish control loop.

### 1.1 Nodes (minimum set)

Keep these from the ROS2 doc:

1) `backyard_controller_node` (C++)
2) `vehicle_interface_node` (C++)
3) `telemetry_jsonl_logger_node` (C++)
4) `ros2_http_gateway_node` (C++) optional, recommended for MiNiFi ops

Add these for LeRobot integration:

5) `learning_observation_node` (C++)
	- subscribes to telemetry + controller status/state
	- builds an ML-friendly observation message
	- publishes observations on a stable topic for policy + recording

6) `action_arbiter_node` (C++)
	- consumes candidate actions from (a) classical controller and (b) learned policy
	- applies safety checks, timeouts, mode switching
	- publishes the single authoritative `CommandIntent` to the vehicle node

7) `episode_recorder_node` (C++)
	- records synchronized (observation, action, metadata) into a dataset directory
	- designed so you can later convert into LeRobot dataset format (or record directly in a compatible layout)

8) `lerobot_policy_runner_node` (often Python, but could later be C++)
	- subscribes to observations
	- runs inference with a trained policy
	- publishes candidate actions

Important note (pragmatic, production reality):

- Many LeRobot workflows are Python-first.
- Your **ROS2 + actuation** can remain C++.
- The “policy runner” can start as Python (`rclpy`) and later migrate to C++ (ONNX Runtime / TensorRT / TFLite) once the policy architecture is stable.

### 1.2 Topic graph (conceptual)

```text
TelemetrySample  --->  learning_observation_node ---> /learning/observation ---> lerobot_policy_runner_node ---> /learning/policy_action
			|                     |                                                              |
			|                     +---> episode_recorder_node (records dataset)                  |
			v                                                                                     v
controller status/events ------------------------------------------------------------> action_arbiter_node ---> CommandIntent ---> vehicle_interface_node
																												classical intent (optional) ----^
```

Where:

- `action_arbiter_node` is the safety-critical “selector” and must be deterministic.
- `lerobot_policy_runner_node` is allowed to be late or unavailable; arbiter falls back.

### 1.3 What LeRobot “replaces” vs what stays

Compared to `SOLUTION_ROS2_JAZZY_CPP.md`, LeRobot does **not** replace ROS2 itself; it replaces the *source of autonomy decisions*.

What stays (production invariants):

- `vehicle_interface_node` remains the only node that talks to MAVSDK/MAVLink/bridge.
- the authoritative actuation contract remains `CommandIntent`.
- the JSONL logger + MiNiFi sidecar remain the telemetry/ops transport boundary.

What changes (autonomy source):

- Instead of *only* the classical FSM/controller producing intents, you add a policy plane that can produce **candidate** actions.
- `action_arbiter_node` becomes the explicit safety gate that chooses between classical vs learned actions.

This is the “smooth transition” mechanism: you can start in classical-only mode, then move to mixed/shadow/policy modes without rewriting transport/logging/ops.

---

## 2) Step-by-step transition plan (ROS2 C++ → LeRobot-enabled ROS2)

### Step 1 — Freeze your learning interface (messages)

You need a stable contract between “robotics system” and “learning system”.

Recommended: add a new msg package `backyard_flyer_learning_msgs` with:

- `LearningObservation.msg`
- `LearningAction.msg`
- `PolicyMode.msg`
- `EpisodeEvent.msg`

Keep it small and numeric (avoid string parsing in hot paths).

### Step 2 — Implement `learning_observation_node` (C++)

Goal: publish a single observation message that is:

- complete enough for learning
- stable across releases
- inexpensive to build

Start minimal: local position/velocity NED + armed/guided + controller state enum + timestamps.

### Step 3 — Implement `episode_recorder_node` (C++)

Goal: record datasets reliably without disturbing actuation.

Design constraints:

- recorder should be on its own callback group / executor thread
- file I/O must not run inside the control loop timer callback

Start with line-delimited JSON (`.jsonl`) per episode and later convert to LeRobot’s preferred dataset layout.

### Step 4 — Implement `action_arbiter_node` (C++)

Goal: keep the vehicle command stream deterministic.

Rules:

- run arbiter at a fixed rate (e.g., 50–100 Hz)
- if policy action is stale (timeout), ignore it
- always allow immediate STOP/LAND from classical safety logic
- clamp commands to mission constraints (altitude band, max step, etc.)

### Step 5 — Add `lerobot_policy_runner_node`

MVP: implement it as a LeRobot/`lerobot-ros` node that:

- subscribes to `/learning/observation`
- runs inference (trained policy)
- publishes `/learning/policy_action`

If you don’t have a trained policy yet:

- publish a “no-op” action
- or keep policy mode disabled and collect data only

### Step 6 — Keep MiNiFi integration unchanged, extend telemetry schema slightly

MiNiFi still tails and ships JSONL logs; this does not change.

What changes:

- logger emits additional event types like `policy_mode_change`, `policy_action_summary`, `episode_boundary`
- MiNiFi can route these separately (alerts, dashboards, dataset tracking)

### Step 7 — Add ops/control-plane hooks (optional, recommended)

Extend the HTTP gateway with endpoints like:

- `POST /v1/policy/mode` (e.g., Classical / Mixed / Policy)
- `POST /v1/episode/start`
- `POST /v1/episode/stop`

MiNiFi can call these via `InvokeHTTP` exactly like in `SOLUTION_MINIFI_CPP_V2.md`.

---

## 3) Follow-along: message contracts (learning)

This section is intentionally minimal and drone-agnostic.

### 3.1 `backyard_flyer_learning_msgs/msg/PolicyMode.msg`

```text
uint8 CLASSICAL=0
uint8 POLICY=1
uint8 MIXED=2

uint8 mode
uint64 t_us
string run_id
```

### 3.2 `backyard_flyer_learning_msgs/msg/LearningObservation.msg`

```text
backyard_flyer_msgs/Ned position_ned
backyard_flyer_msgs/Ned velocity_ned

bool armed
bool guided

uint8 controller_state   # map your FSM states to small ints
bool in_mission

uint64 t_us
string run_id
string vehicle_id
```

### 3.3 `backyard_flyer_learning_msgs/msg/LearningAction.msg`

Option A (simplest transition): reuse the same semantics as `CommandIntent`.

```text
uint8 type
float64 north
float64 east
float64 down
float64 heading_rad
float64 takeoff_altitude_m

float64 policy_confidence   # optional
uint64 t_us
string run_id
```

### 3.4 `backyard_flyer_learning_msgs/msg/EpisodeEvent.msg`

```text
uint8 EPISODE_START=0
uint8 EPISODE_END=1

uint8 type
string episode_id
string run_id
uint64 t_us
string reason
```

---

## 4) Follow-along: `learning_observation_node` (C++ skeleton)

This node translates your existing ROS2 telemetry/status topics into `LearningObservation`.

### 4.1 Header: `learning_observation_node.hpp`

```cpp
#pragma once

#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "backyard_flyer_msgs/msg/telemetry_sample.hpp"
#include "backyard_flyer_msgs/msg/controller_status.hpp"

#include "backyard_flyer_learning_msgs/msg/learning_observation.hpp"

namespace backyard_flyer_learning {

class LearningObservationNode final : public rclcpp::Node {
 public:
	explicit LearningObservationNode(const rclcpp::NodeOptions& options);

 private:
	void onTelemetry(const backyard_flyer_msgs::msg::TelemetrySample::SharedPtr msg);
	void onStatus(const backyard_flyer_msgs::msg::ControllerStatus::SharedPtr msg);
	void onTick();

	std::mutex mutex_;
	backyard_flyer_msgs::msg::TelemetrySample last_telemetry_{};
	backyard_flyer_msgs::msg::ControllerStatus last_status_{};
	bool have_telemetry_{false};
	bool have_status_{false};

	rclcpp::Subscription<backyard_flyer_msgs::msg::TelemetrySample>::SharedPtr telemetry_sub_;
	rclcpp::Subscription<backyard_flyer_msgs::msg::ControllerStatus>::SharedPtr status_sub_;
	rclcpp::Publisher<backyard_flyer_learning_msgs::msg::LearningObservation>::SharedPtr obs_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace backyard_flyer_learning
```

### 4.2 Source: `learning_observation_node.cpp`

```cpp
#include "learning_observation_node.hpp"

namespace backyard_flyer_learning {

LearningObservationNode::LearningObservationNode(const rclcpp::NodeOptions& options)
		: rclcpp::Node("learning_observation_node", options) {
	const double publish_rate_hz = this->declare_parameter<double>("publish_rate_hz", 50.0);

	obs_pub_ = this->create_publisher<backyard_flyer_learning_msgs::msg::LearningObservation>(
			"/learning/observation", rclcpp::SensorDataQoS());

	telemetry_sub_ = this->create_subscription<backyard_flyer_msgs::msg::TelemetrySample>(
			"/telemetry/sample", rclcpp::SensorDataQoS(),
			[this](const backyard_flyer_msgs::msg::TelemetrySample::SharedPtr msg) { onTelemetry(msg); });

	status_sub_ = this->create_subscription<backyard_flyer_msgs::msg::ControllerStatus>(
			"/controller/status", rclcpp::SystemDefaultsQoS(),
			[this](const backyard_flyer_msgs::msg::ControllerStatus::SharedPtr msg) { onStatus(msg); });

	const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
	timer_ = this->create_wall_timer(period, [this]() { onTick(); });
}

void LearningObservationNode::onTelemetry(const backyard_flyer_msgs::msg::TelemetrySample::SharedPtr msg) {
	std::lock_guard<std::mutex> lock(mutex_);
	last_telemetry_ = *msg;
	have_telemetry_ = true;
}

void LearningObservationNode::onStatus(const backyard_flyer_msgs::msg::ControllerStatus::SharedPtr msg) {
	std::lock_guard<std::mutex> lock(mutex_);
	last_status_ = *msg;
	have_status_ = true;
}

void LearningObservationNode::onTick() {
	backyard_flyer_msgs::msg::TelemetrySample telemetry;
	backyard_flyer_msgs::msg::ControllerStatus status;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (!have_telemetry_ || !have_status_) {
			return;
		}
		telemetry = last_telemetry_;
		status = last_status_;
	}

	backyard_flyer_learning_msgs::msg::LearningObservation obs;
	obs.position_ned = telemetry.position_ned;
	obs.velocity_ned = telemetry.velocity_ned;
	obs.armed = telemetry.armed;
	obs.guided = telemetry.guided;

	// Map controller_state string to a small int in your codebase.
	// For MVP: keep a small mapping table in this node.
	obs.controller_state = 0;
	obs.in_mission = status.in_mission;
	obs.t_us = telemetry.t_us;
	obs.run_id = status.run_id;
	obs.vehicle_id = status.vehicle_id;

	obs_pub_->publish(obs);
}

}  // namespace backyard_flyer_learning
```

Design notes:

- Observation publication is timer-driven to avoid coupling to telemetry burst rate.
- This node is not safety critical; it can drop messages.

---

## 5) Follow-along: `action_arbiter_node` (C++ skeleton)

The arbiter selects which action stream drives the vehicle.

### 5.1 Intent sources

- Classical controller intent: `/controller/intent` (type `CommandIntent`)
- Policy action: `/learning/policy_action` (type `LearningAction`)

### 5.2 Arbiter output

- Authoritative intent: `vehicle/command_intent` (type `CommandIntent`)

Note: this matches the topic used by the baseline ROS2 guide in `SOLUTION_ROS2_JAZZY_CPP.md`.

### 5.3 Source sketch

```cpp
#pragma once

#include <chrono>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "backyard_flyer_msgs/msg/command_intent.hpp"
#include "backyard_flyer_learning_msgs/msg/learning_action.hpp"
#include "backyard_flyer_learning_msgs/msg/policy_mode.hpp"

namespace backyard_flyer_learning {

class ActionArbiterNode final : public rclcpp::Node {
 public:
	explicit ActionArbiterNode(const rclcpp::NodeOptions& options);

 private:
	void onClassicalIntent(const backyard_flyer_msgs::msg::CommandIntent::SharedPtr msg);
	void onPolicyAction(const backyard_flyer_learning_msgs::msg::LearningAction::SharedPtr msg);
	void onPolicyMode(const backyard_flyer_learning_msgs::msg::PolicyMode::SharedPtr msg);
	void onTick();

	backyard_flyer_msgs::msg::CommandIntent chooseIntentLocked(
			const rclcpp::Time& now, const backyard_flyer_msgs::msg::CommandIntent& classical,
			const backyard_flyer_learning_msgs::msg::LearningAction& policy) const;

	mutable std::mutex mutex_;
	backyard_flyer_msgs::msg::CommandIntent last_classical_{};
	backyard_flyer_learning_msgs::msg::LearningAction last_policy_{};
	rclcpp::Time last_policy_rx_{0, 0, RCL_ROS_TIME};
	bool have_classical_{false};
	bool have_policy_{false};

	uint8_t mode_{backyard_flyer_learning_msgs::msg::PolicyMode::CLASSICAL};
	std::chrono::milliseconds policy_timeout_{100};

	rclcpp::Subscription<backyard_flyer_msgs::msg::CommandIntent>::SharedPtr classical_sub_;
	rclcpp::Subscription<backyard_flyer_learning_msgs::msg::LearningAction>::SharedPtr policy_sub_;
	rclcpp::Subscription<backyard_flyer_learning_msgs::msg::PolicyMode>::SharedPtr mode_sub_;
	rclcpp::Publisher<backyard_flyer_msgs::msg::CommandIntent>::SharedPtr intent_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace backyard_flyer_learning
```

Implementation sketch (core loop):

```cpp
void ActionArbiterNode::onTick() {
	const auto now = this->now();

	backyard_flyer_msgs::msg::CommandIntent classical;
	backyard_flyer_learning_msgs::msg::LearningAction policy;
	rclcpp::Time policy_rx;
	uint8_t mode;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (!have_classical_) return;
		classical = last_classical_;
		policy = last_policy_;
		policy_rx = last_policy_rx_;
		mode = mode_;
	}

	if (mode == backyard_flyer_learning_msgs::msg::PolicyMode::CLASSICAL) {
		intent_pub_->publish(classical);
		return;
	}

	const bool policy_fresh = have_policy_ && ((now - policy_rx) < rclcpp::Duration(policy_timeout_));
	if (!policy_fresh) {
		// Fallback: never stall vehicle commands waiting on policy.
		intent_pub_->publish(classical);
		return;
	}

	// Convert policy action to CommandIntent and clamp / safety-filter here.
	backyard_flyer_msgs::msg::CommandIntent chosen = classical;
	chosen.type = policy.type;
	chosen.north = policy.north;
	chosen.east = policy.east;
	chosen.down = policy.down;
	chosen.heading_rad = policy.heading_rad;
	chosen.takeoff_altitude_m = policy.takeoff_altitude_m;

	intent_pub_->publish(chosen);
}
```

Safety notes (important):

- In `MIXED` mode, prefer “classical wins” for STOP/LAND.
- Clamp any position targets to a safe box region and altitude band.
- Always implement a policy timeout fallback.

---

## 6) MiNiFi interaction (unchanged mechanics, expanded content)

Your MiNiFi approach stays the same:

- ROS2 logger writes JSONL under `Logs/`
- MiNiFi tails JSONL and ships it
- MiNiFi triggers ops via HTTP gateway

### 6.1 JSONL event extensions (examples)

Add event types alongside the existing ones from `SOLUTION_MINIFI_CPP.md`:

```json
{
	"event_type": "policy_mode_change",
	"t_us": 1700000002123456,
	"vehicle_id": "sim-01",
	"run_id": "2026-02-23T01-23-45Z",
	"mode": "POLICY"
}
```

```json
{
	"event_type": "episode_boundary",
	"t_us": 1700000003123456,
	"vehicle_id": "sim-01",
	"run_id": "2026-02-23T01-23-45Z",
	"episode_id": "ep-000042",
	"boundary": "START"
}
```

MiNiFi routing ideas (still optional):

- route `episode_boundary` to dataset tracking storage
- route `policy_mode_change` to ops alerts
- keep high-rate samples throttled (as already described)

### 6.2 Control-plane hooks from MiNiFi V2

Just like `POST /v1/mission/start`, add endpoints:

- `POST /v1/policy/mode`
- `POST /v1/episode/start`
- `POST /v1/episode/stop`

MiNiFi uses built-in `InvokeHTTP` to call these.

---

## 7) How this enables future AI autonomy (DL/RL/LLM/LVM)

Once you have the above interfaces, you can transition in layers without rewriting your production ROS2 plumbing.

### 7.1 Safe incremental path

1) **Data only**: classical controller flies; you record observations/actions.
2) **Offline imitation**: train policy to mimic classical controller.
3) **Shadow mode**: policy runs but arbiter ignores it; compare actions/logs.
4) **Mixed mode**: policy drives some intents; classical safety overrides.
5) **Policy mode**: policy drives most decisions; classical remains as safety fallback.

### 7.2 Where LLM/LVM fits (without breaking real-time)

Use LLM/LVM for *high-level* tasks that are not hard real-time:

- generating waypoint patterns from a natural-language mission
- interpreting operator instructions
- summarizing run anomalies

Keep LLM outputs as **high-level intents** that your classical/controller layer validates and translates.

---

## 8) Acceptance criteria (for this transition)

- In `CLASSICAL` mode, behavior matches `SOLUTION_ROS2_JAZZY_CPP.md`.
- Enabling LeRobot nodes does not measurably increase control-loop jitter.
- If policy runner is killed/unavailable, vehicle commands continue safely (fallback).
- MiNiFi continues to ship telemetry/status; new learning events are routed successfully.

