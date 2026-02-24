
# Backyard Flyer — ROS2 Jazzy C++ Production Approach (Custom Nodes, Follow-Along Guide)

This document explains how to transition from the **modular, class-based C++ approach** (`SOLUTION_MODULAR_CPP.md`) to a **ROS2 Jazzy C++ production approach** using **custom ROS2 C++ nodes**.

It also explains how this ROS2 approach interacts with the existing **MiNiFi C++ edge pipeline** described in:

- `SOLUTION_MINIFI_CPP.md` (telemetry shipping)
- `SOLUTION_MINIFI_CPP_V2.md` (telemetry + controller ops/control-plane)

Inputs re-analyzed:

- `README.md` (mission requirements, state machine)
- `SOLUTION_DESIGN_CPP.md` (real-time framing, concurrency model)
- `SOLUTION_DESIGN_FOLDER_STRUCTURE_CPP.md` (staged repo structure)
- `SOLUTION_MODULAR_CPP.md` (core controller + wiring)
- `SOLUTION_MINIFI_CPP.md` and `SOLUTION_MINIFI_CPP_V2.md` (MiNiFi boundary + JSONL telemetry contracts)

Mission reminder:

- Fly a **10m box** at **3m altitude**, then land/disarm/end mission.
- Local frame is **NED**; 3m altitude corresponds to `down = -3.0`.

---

## 0) Can ROS2 Jazzy C++ handle “real-time”?

ROS2 in C++ can be used in **real-time-ish** systems, and it’s a common production choice for robotics. But the same caution applies as with MiNiFi:

- “Real-time” is not automatic.
- You must design for bounded latency (executor choice, callback design, QoS, memory allocation patterns, OS config).

### 0.1 What ROS2 replaces from the modular C++ approach

In `SOLUTION_MODULAR_CPP.md`, you had:

- a core library (`BackyardFlyerController`, guards, planner, config)
- a single app that wires threads + transport + logging

ROS2 replaces the “wiring app” with:

- a set of nodes with explicit interfaces (topics/services)
- an executor that schedules callbacks
- standard parameterization, lifecycle, composition

The core **flight logic** (FSM/planner/guards) should remain a normal C++ library.

### 0.2 What “real-time” means here (same as `SOLUTION_DESIGN_CPP.md`)

- fixed-rate control cadence (example: 100 Hz)
- minimal jitter in the control decision loop
- explicit data handoff from telemetry I/O to control loop
- avoid allocations in the hot path

In ROS2, you typically express “fixed-rate loop” via a timer callback, and you keep callbacks short and predictable.

---

## 1) Target end-state architecture (ROS2 + MiNiFi)

We keep the safe production boundary:

- **Actuation/data plane**: the controller node makes decisions and emits command intents
- **Vehicle transport node**: talks MAVSDK/MAVLink/bridge and executes command intents
- **Telemetry plane**: ROS2 topics carry telemetry and events
- **MiNiFi plane**: ships JSONL logs and optionally triggers ops workflows

### 1.1 Nodes (custom)

Minimum set (recommended):

1) `backyard_controller_node` (C++)
	- wraps `BackyardFlyerController` and runs a fixed-rate timer
	- subscribes to telemetry
	- publishes command intents + status/events

2) `vehicle_interface_node` (C++)
	- subscribes to command intents
	- sends commands to the simulator/autopilot (MAVSDK, MAVLink C, or a bridge)
	- publishes telemetry (or simply relays telemetry from transport)

3) `telemetry_jsonl_logger_node` (C++)
	- subscribes to telemetry/events/status
	- writes JSONL to `Logs/telemetry.jsonl` and `Logs/controller_status.jsonl`
	- keeps the JSON contract consistent with the MiNiFi docs

Optional node (for MiNiFi control-plane integration):

4) `ros2_http_gateway_node` (C++)
	- exposes localhost HTTP endpoints (start/stop/config)
	- calls ROS2 services on `backyard_controller_node`
	- allows MiNiFi to use built-in `InvokeHTTP` (as in `SOLUTION_MINIFI_CPP_V2.md`)

### 1.2 Why we don’t make MiNiFi “speak ROS2” directly

MiNiFi doesn’t natively speak DDS/ROS2.

You *can* script `ros2 service call` via `ExecuteProcess`, but it’s brittle.

So the clean integration is:

- ROS2 side provides an HTTP control-plane gateway.
- MiNiFi triggers HTTP calls and ships JSONL logs.

---

## 2) Step-by-step transition plan

### Step 1 — Keep `backyard_flyer_core` as a plain C++ library

Do not rewrite the FSM into ROS2 callbacks directly.

Keep your deterministic modules from `SOLUTION_MODULAR_CPP.md`:

- `Config`, `TelemetrySample`, `BoxPlanner`, `guards`, `BackyardFlyerController`

The core library should have no ROS2 headers.

### Step 2 — Define ROS2 message contracts (topics)

Create a ROS2 interface package (messages) so nodes can communicate without sharing headers.

Recommended message set:

- `TelemetrySample.msg`
- `CommandIntent.msg`
- `ControllerStatus.msg`
- `TelemetryEvent.msg` (optional but useful)

### Step 3 — Implement the controller node (timer + subscriptions)

The controller node:

- subscribes to `TelemetrySample`
- stores “latest sample” atomically/mutex-protected
- runs a timer at `control_rate_hz` (e.g., 100 Hz)
- calls `controller.UpdateTelemetry(sample)` and `controller.Tick()`
- publishes command intents on transitions
- publishes status and/or telemetry events

This is the ROS2 version of `SOLUTION_DESIGN_CPP.md` Model B (telemetry thread writes, control loop reads), expressed as:

- DDS receive thread → subscription callback updates latest sample
- control timer callback reads latest sample and ticks

### Step 4 — Implement vehicle interface node (executes commands)

The vehicle node takes `CommandIntent` messages and executes them using a selected transport:

- MVP: a bridge to Udacity simulator
- Production-ish: MAVSDK

This node replaces `IVehicle` implementation classes.

### Step 5 — Implement JSONL logger node (MiNiFi compatibility)

The logger node writes JSONL files matching `SOLUTION_MINIFI_CPP.md` and `SOLUTION_MINIFI_CPP_V2.md`.

MiNiFi can then:

- `TailFile` those logs
- parse/validate/rate-limit
- ship to NiFi/platform
- trigger start/stop via HTTP gateway

### Step 6 — Add an HTTP gateway (optional but recommended for MiNiFi ops)

Expose:

- `POST /v1/mission/start`
- `POST /v1/mission/stop`
- `POST /v1/config` (optional)

The gateway node translates these into ROS2 service calls.

---

## 3) Follow-along: ROS2 workspace and packages

This section is written so you can create the files manually.

### 3.1 Workspace layout (conceptual)

```text
ros2_ws/
  src/
	 backyard_flyer_core/           # your existing core library (no ROS2)
	 backyard_flyer_msgs/           # msg package
	 backyard_flyer_nodes/          # controller + logger nodes
	 backyard_flyer_vehicle/        # vehicle interface node (MAVSDK/bridge)
	 backyard_flyer_http_gateway/   # optional
```

You will also keep your MiNiFi config under the main repo’s `edge/minifi-cpp/` (see the folder-structure doc).

---

## 4) Follow-along: `backyard_flyer_msgs` (interfaces)

### 4.1 `backyard_flyer_msgs/msg/Ned.msg`

```text
float64 north
float64 east
float64 down
```

### 4.2 `backyard_flyer_msgs/msg/TelemetrySample.msg`

```text
backyard_flyer_msgs/Ned position_ned
backyard_flyer_msgs/Ned velocity_ned
bool armed
bool guided
uint64 t_us
```

### 4.3 `backyard_flyer_msgs/msg/CommandIntent.msg`

Use a small enum-like integer to avoid string parsing in the hot path.

```text
uint8 TAKE_CONTROL=0
uint8 RELEASE_CONTROL=1
uint8 ARM=2
uint8 DISARM=3
uint8 TAKEOFF=4
uint8 LAND=5
uint8 CMD_POSITION=6
uint8 STOP=7

uint8 type
float64 north
float64 east
float64 down
float64 heading_rad
float64 takeoff_altitude_m
```

Notes:

- `takeoff_altitude_m` is used only when `type == TAKEOFF`.
- `(north,east,down,heading_rad)` are used only when `type == CMD_POSITION`.

### 4.4 `backyard_flyer_msgs/msg/ControllerStatus.msg`

```text
string controller_state
bool in_mission
float64 loop_rate_hz
float64 loop_jitter_ms_p95
uint64 t_us
string vehicle_id
string run_id
```

### 4.5 `backyard_flyer_msgs/msg/TelemetryEvent.msg` (optional)

If you want explicit events rather than reconstructing from status:

```text
string event_type        # state_transition, command_waypoint, mission_summary, etc.
string vehicle_id
string run_id
uint64 t_us
string payload_json      # keep it simple for MVP
```

---

## 5) Follow-along: `backyard_controller_node` (C++)

This node owns the fixed-rate loop.

### 5.1 Core idea

- Subscription callback: store latest telemetry sample
- Timer callback @ `control_rate_hz`: tick the controller

### 5.2 `backyard_flyer_nodes/include/backyard_controller_node.hpp`

```cpp
#pragma once

#include <mutex>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include "backyard_flyer_msgs/msg/command_intent.hpp"
#include "backyard_flyer_msgs/msg/controller_status.hpp"
#include "backyard_flyer_msgs/msg/telemetry_sample.hpp"

#include "backyard/config.hpp"
#include "backyard/controller.hpp"
#include "backyard/telemetry.hpp"
#include "backyard/vehicle.hpp"

namespace backyard_flyer_nodes {

class RosVehicleAdapter final : public backyard::IVehicle {
 public:
  explicit RosVehicleAdapter(rclcpp::Publisher<backyard_flyer_msgs::msg::CommandIntent>::SharedPtr pub)
      : pub_(std::move(pub)) {}

  void TakeControl() override;
  void ReleaseControl() override;
  void Arm() override;
  void Disarm() override;
  void Takeoff(double altitude_m) override;
  void Land() override;
  void CmdPosition(double north, double east, double down, double heading_rad) override;
  void Stop() override;

 private:
  void publishType(uint8_t type);

  rclcpp::Publisher<backyard_flyer_msgs::msg::CommandIntent>::SharedPtr pub_;
};

class BackyardControllerNode final : public rclcpp::Node {
 public:
  explicit BackyardControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void onTelemetry(const backyard_flyer_msgs::msg::TelemetrySample& msg);
  void onTick();

  backyard::TelemetrySample toCoreSample(const backyard_flyer_msgs::msg::TelemetrySample& msg) const;
  void publishStatus(uint64_t t_us);

  // Parameters
  backyard::Config cfg_;

  // Publish-backed adapter that satisfies the existing IVehicle seam.
  // This lets you keep your core controller unchanged.
  std::unique_ptr<RosVehicleAdapter> vehicle_adapter_;
  std::unique_ptr<backyard::BackyardFlyerController> controller_;

  // Latest telemetry sample
  std::mutex sample_mu_;
  std::optional<backyard::TelemetrySample> latest_;

  // Publishers/subscribers
  rclcpp::Subscription<backyard_flyer_msgs::msg::TelemetrySample>::SharedPtr telemetry_sub_;
  rclcpp::Publisher<backyard_flyer_msgs::msg::CommandIntent>::SharedPtr cmd_pub_;
  rclcpp::Publisher<backyard_flyer_msgs::msg::ControllerStatus>::SharedPtr status_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Metadata
  std::string vehicle_id_;
  std::string run_id_;
};

}  // namespace backyard_flyer_nodes
```

### 5.3 `backyard_flyer_nodes/src/backyard_controller_node.cpp` (skeleton)

```cpp
#include "backyard_controller_node.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace backyard_flyer_nodes {

void RosVehicleAdapter::publishType(uint8_t type) {
  backyard_flyer_msgs::msg::CommandIntent msg;
  msg.type = type;
  pub_->publish(msg);
}

void RosVehicleAdapter::TakeControl() { publishType(backyard_flyer_msgs::msg::CommandIntent::TAKE_CONTROL); }
void RosVehicleAdapter::ReleaseControl() { publishType(backyard_flyer_msgs::msg::CommandIntent::RELEASE_CONTROL); }
void RosVehicleAdapter::Arm() { publishType(backyard_flyer_msgs::msg::CommandIntent::ARM); }
void RosVehicleAdapter::Disarm() { publishType(backyard_flyer_msgs::msg::CommandIntent::DISARM); }
void RosVehicleAdapter::Land() { publishType(backyard_flyer_msgs::msg::CommandIntent::LAND); }
void RosVehicleAdapter::Stop() { publishType(backyard_flyer_msgs::msg::CommandIntent::STOP); }

void RosVehicleAdapter::Takeoff(double altitude_m) {
  backyard_flyer_msgs::msg::CommandIntent msg;
  msg.type = backyard_flyer_msgs::msg::CommandIntent::TAKEOFF;
  msg.takeoff_altitude_m = altitude_m;
  pub_->publish(msg);
}

void RosVehicleAdapter::CmdPosition(double north, double east, double down, double heading_rad) {
  backyard_flyer_msgs::msg::CommandIntent msg;
  msg.type = backyard_flyer_msgs::msg::CommandIntent::CMD_POSITION;
  msg.north = north;
  msg.east = east;
  msg.down = down;
  msg.heading_rad = heading_rad;
  pub_->publish(msg);
}

BackyardControllerNode::BackyardControllerNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("backyard_controller", options) {
  // Parameters (MVP)
  this->declare_parameter("vehicle_id", "sim-01");
  this->declare_parameter("run_id", "dev");
  this->declare_parameter("control_rate_hz", 100.0);
  this->declare_parameter("target_altitude_m", 3.0);
  this->declare_parameter("box_size_m", 10.0);

  vehicle_id_ = this->get_parameter("vehicle_id").as_string();
  run_id_ = this->get_parameter("run_id").as_string();

  cfg_.control_rate_hz = this->get_parameter("control_rate_hz").as_double();
  cfg_.target_altitude_m = this->get_parameter("target_altitude_m").as_double();
  cfg_.box_size_m = this->get_parameter("box_size_m").as_double();

  telemetry_sub_ = this->create_subscription<backyard_flyer_msgs::msg::TelemetrySample>(
		"telemetry/sample", rclcpp::SensorDataQoS(),
		[this](const backyard_flyer_msgs::msg::TelemetrySample& msg) { this->onTelemetry(msg); });

  cmd_pub_ = this->create_publisher<backyard_flyer_msgs::msg::CommandIntent>("vehicle/command_intent", 10);
  status_pub_ = this->create_publisher<backyard_flyer_msgs::msg::ControllerStatus>("controller/status", 10);

  // Construct adapter first, then the core controller.
  vehicle_adapter_ = std::make_unique<RosVehicleAdapter>(cmd_pub_);
  controller_ = std::make_unique<backyard::BackyardFlyerController>(cfg_, *vehicle_adapter_);

  const auto period = std::chrono::duration<double>(1.0 / cfg_.control_rate_hz);
  timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
											 [this]() { this->onTick(); });
}

void BackyardControllerNode::onTelemetry(const backyard_flyer_msgs::msg::TelemetrySample& msg) {
  std::lock_guard<std::mutex> lk(sample_mu_);
  latest_ = toCoreSample(msg);
}

void BackyardControllerNode::onTick() {
  backyard::TelemetrySample local;
  {
	 std::lock_guard<std::mutex> lk(sample_mu_);
	 if (!latest_) return;
	 local = *latest_;
  }

  controller_->UpdateTelemetry(local);
  controller_->Tick();

  publishStatus(local.t_us);
}

backyard::TelemetrySample BackyardControllerNode::toCoreSample(const backyard_flyer_msgs::msg::TelemetrySample& msg) const {
  backyard::TelemetrySample s;
  s.position_ned = backyard::NED{msg.position_ned.north, msg.position_ned.east, msg.position_ned.down};
  s.velocity_ned = backyard::NED{msg.velocity_ned.north, msg.velocity_ned.east, msg.velocity_ned.down};
  s.armed = msg.armed;
  s.guided = msg.guided;
  s.t_us = msg.t_us;
  return s;
}

void BackyardControllerNode::publishStatus(uint64_t t_us) {
  backyard_flyer_msgs::msg::ControllerStatus st;
  st.controller_state = "Unknown";
  st.in_mission = true;
  st.loop_rate_hz = cfg_.control_rate_hz;
  st.loop_jitter_ms_p95 = 0.0;
  st.t_us = t_us;
  st.vehicle_id = vehicle_id_;
  st.run_id = run_id_;
  status_pub_->publish(st);
}

}  // namespace backyard_flyer_nodes
```

Follow-along note:

The skeleton above intentionally stops short of “publishing intents from the controller”, because your current `BackyardFlyerController` is written to call `IVehicle` methods directly.

Two practical ways to bridge that gap:

1) **Replace `IVehicle` with a ROS2 publisher-backed adapter**
	- Implement `RosVehicleAdapter : IVehicle` that publishes `CommandIntent` messages.
	- Keep the controller code unchanged.

2) **Refactor controller transitions to emit intents**
	- More invasive, but removes the `IVehicle` dependency inside the node.

For follow-along, option (1) is the smallest change.

---

## 6) Follow-along: `vehicle_interface_node` (C++)

This node subscribes to `vehicle/command_intent` and executes commands using your chosen transport.

Transport options (from `SOLUTION_DESIGN_CPP.md`):

- bridge (migration-friendly for Udacity sim)
- MAVSDK (production-ish)
- MAVLink C (lower-level)

For learning, keep it simple:

- implement a stub executor first that just logs intents
- then swap in MAVSDK/bridge

---

## 7) Follow-along: `telemetry_jsonl_logger_node` (C++)

Goal: keep MiNiFi integration identical to V1/V2.

This node subscribes to:

- `controller/status`
- (optional) `telemetry/events`

And writes:

- `Logs/controller_status.jsonl`
- `Logs/telemetry.jsonl`

So MiNiFi can use the exact same `TailFile → Parse → RateLimit → Route → PutTCP/InvokeHTTP` flows.

---

## 8) How ROS2 and MiNiFi interact (recommended pattern)

### 8.1 Telemetry shipping

- ROS2 nodes produce telemetry/status
- logger node writes JSONL
- MiNiFi tails JSONL and ships to platform

Why this is still good even if ROS2 already has networking:

- MiNiFi gives you disk-backed buffering/backpressure and easy routing
- it decouples platform connectivity from your robotics runtime

### 8.2 Ops/control-plane

If you want MiNiFi to trigger start/stop/config:

- run `ros2_http_gateway_node` on localhost
- MiNiFi uses built-in `InvokeHTTP`

Alternative (not recommended for production):

- MiNiFi uses `ExecuteProcess` to run `ros2 service call ...`

---

## 9) Real-time-ish best practices for ROS2 in this project

Keep this pragmatic and MVP-friendly:

1) Use a fixed-rate timer for the controller tick.
2) Keep subscription callbacks “copy latest sample only”.
3) Avoid allocations inside hot callbacks where possible.
4) Prefer simple QoS:
	- telemetry topic: `SensorDataQoS()`
	- command topic: reliable QoS with small queue
5) If you need tighter bounds later:
	- PREEMPT_RT kernel
	- CPU affinity / priorities
	- static executors / callback group separation
	- loaned messages where supported

---

## 10) What “done” looks like (ROS2 acceptance criteria)

Functional:

- Drone flies the 10m box at 3m and lands/disarms.

Engineering:

- Controller tick runs at configured rate without command spam.
- All actuation happens through a single command channel (`CommandIntent`).
- JSONL logs match the MiNiFi processors’ expected schema.
- MiNiFi can ship telemetry and optionally trigger start/stop via HTTP gateway.

