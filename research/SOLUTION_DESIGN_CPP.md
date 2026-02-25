# FCND Backyard Flyer — Solution Design (C++ Real-Time Transition)

This document explains how to transition the current Python design (`SOLUTION_DESIGN_PY.md`) into a **C++ implementation** optimized for **more real-time performance**, while keeping the same functional requirement from `README.md`:

- Fly a **10m box** at **3m altitude**, then land and end mission.
- Use an **event-driven state machine** based on incoming telemetry updates.

It also includes:

- A pragmatic **MVP → Modular** evolution plan for C++.
- A **MiNiFi C++ → NiFi** telemetry evolution approach.

---

## 1) What changes when moving Python → C++ (real-time framing)

The Python solution is already architecturally “correct” for this task (callbacks + state machine + tolerances). The main differences when transitioning to C++ for performance are:

1) **Control-loop determinism**
	 - Reduce unpredictable latency (GC, interpreter overhead).
	 - Use fixed-rate processing (e.g., 50–200 Hz) for guards/actions.

2) **Concurrency model becomes explicit**
	 - You must safely move data from a telemetry receiver thread into the controller logic.

3) **Memory management and allocations matter**
	 - Avoid per-message heap allocations in the hot path.
	 - Prefer pre-allocated containers and POD-style structs.

4) **Separation of concerns is still the same**
	 - Transition actions: send commands once.
	 - Guards: check thresholds/tolerances on new measurements.

Crucially: “Real-time performance based” does *not* mean “NiFi in the control loop.” The control loop remains local; NiFi/MiNiFi is for telemetry.

---

## 2) Requirements recap (from README.md)

- Implement a subclass/controller using the Drone API concepts:
	- Telemetry: local position (NED), local velocity, armed/guided state.
	- Commands: arm/disarm, takeoff, cmd_position, land, take/release control, stop.
- Use callbacks/listeners (event-driven).
- The six logical states:
	- MANUAL → ARMING → TAKEOFF → WAYPOINT → LANDING → DISARMING → MANUAL

Reference frame reminder:

- Local frame is **NED**.
- Altitude of 3m above ground corresponds to `down = -3.0` **in NED telemetry**.

Command reminder (to prevent a common regression when bridging back to UdaciDrone/sim APIs):

- Some command APIs take **altitude-up**, not NED down. For example, UdaciDrone `cmd_position(north, east, altitude, heading)` expects altitude-up.
- Keep your C++ core consistent (e.g., NED everywhere internally) and do the translation in the vehicle/adapter layer.

---

## 3) Transition strategy overview (step-by-step plan)

### Step 1 — Freeze the behavioral spec

Before touching C++:

- Document guard thresholds (altitude ratio, waypoint tolerance, landing “ready to disarm”).
- Document waypoint order (the square corners) and acceptance criteria (it flies the box).

This makes it possible to validate the C++ rewrite by comparing logs and observed behavior.

### Step 2 — Decide the comms layer in C++

In Python, UdaciDrone abstracts MAVLink. In C++, you need an equivalent layer. Common approaches:

- **Use MAVSDK (recommended for modern C++ integrations)**
	- Provides telemetry subscriptions and action/mission APIs.
	- Great for real drone / SITL use cases.

- **Use MAVLink C library directly**
	- Maximum control and minimal overhead.
	- More implementation effort (parsing, connection, timeouts, threading).

- **Keep simulator interface stable by creating a small TCP bridge**
	- If the Unity simulator expects UdaciDrone specifics, you can run a Python “bridge” that translates to a C++-friendly format.
	- This is often the fastest path to validate control logic in C++ without fighting simulator assumptions.

MVP recommendation:

- If you are still using the Udacity Unity sim: start with the **bridge** approach.
- If your goal is true real-time + production-ish: choose **MAVSDK**.

### Step 3 — Implement the same state machine core in C++

Port the Python architecture *as-is*:

- A `FlightState` enum.
- A `Config` struct.
- A `BoxPlanner` that generates 4 waypoints.
- A `Guards` module with pure functions.
- A `Controller`/`FSM` class that processes telemetry updates and triggers transitions.

### Step 4 — Choose the event handling model

In C++, you typically pick one of these:

**Model A: Telemetry callbacks directly run guards/transitions**

- Pros: simplest.
- Cons: callback execution time may vary; you can accidentally block telemetry threads.

**Model B (recommended for “more real-time”): Telemetry thread writes, control loop reads**

- Telemetry callbacks only update “latest sample.”
- A dedicated control thread runs at fixed rate (e.g., 100 Hz), consumes latest sample, runs guards and transitions.
- Pros: deterministic processing cadence; protects telemetry transport threads.

For performance and predictability, use Model B.

---

## 4) C++ MVP architecture (minimal but real-time aware)

### 4.1 Core data structures (no allocations in hot path)

Use simple structs:

```cpp
struct NED {
	double north{0.0};
	double east{0.0};
	double down{0.0};
};

struct TelemetrySample {
	NED position_ned;
	NED velocity_ned;
	bool armed{false};
	bool guided{false};
	uint64_t t_us{0};
};

struct Config {
	double box_size_m{10.0};
	double target_altitude_m{3.0};
	double pos_tol_m{0.5};
	double vel_xy_tol_mps{0.5};
	double ground_down_tol_m{0.1};
	double takeoff_ratio{0.95};
	double heading_rad{0.0};
	double control_rate_hz{100.0};
};
```

### 4.2 State machine (transitions vs guards)

Keep the same separation:

- **Transitions**: send a command once + update state.
- **Guards**: boolean checks using `TelemetrySample` and config.

```cpp
enum class FlightState {
	Manual,
	Arming,
	Takeoff,
	Waypoint,
	Landing,
	Disarming
};
```

### 4.3 Planner (box waypoints)

Use a fixed-size container to avoid allocations.

```cpp
#include <array>

class BoxPlanner {
 public:
	explicit BoxPlanner(const Config& cfg) : cfg_(cfg) {}

	std::array<NED, 4> BuildBox(double n0, double e0) const {
		const double d = -cfg_.target_altitude_m;
		const double s = cfg_.box_size_m;
		return {NED{n0 + s, e0 + 0.0, d},
						NED{n0 + s, e0 + s,   d},
						NED{n0 + 0.0, e0 + s, d},
						NED{n0 + 0.0, e0 + 0.0, d}};
	}

 private:
	const Config& cfg_;
};
```

### 4.4 Guards (pure functions)

```cpp
#include <cmath>

inline bool AltitudeReached(const TelemetrySample& s, const Config& cfg) {
	const double altitude = -s.position_ned.down;
	return altitude >= cfg.takeoff_ratio * cfg.target_altitude_m;
}

inline double Norm2D(double x, double y) {
	return std::sqrt(x*x + y*y);
}

inline bool WaypointReachedXY(const TelemetrySample& s, const Config& cfg, const NED& target) {
	const double dn = s.position_ned.north - target.north;
	const double de = s.position_ned.east  - target.east;
	return Norm2D(dn, de) <= cfg.pos_tol_m;
}

inline bool ReadyToDisarm(const TelemetrySample& s, const Config& cfg) {
	const bool near_ground = std::abs(s.position_ned.down) <= cfg.ground_down_tol_m;
	const double vxy = Norm2D(s.velocity_ned.north, s.velocity_ned.east);
	const bool stopped = vxy <= cfg.vel_xy_tol_mps;
	return near_ground && stopped;
}
```

### 4.5 Vehicle/transport interface (adapter)

Define a small interface so the FSM is independent of MAVSDK/MAVLink/sim bridge:

```cpp
class IVehicle {
 public:
	virtual ~IVehicle() = default;
	virtual void TakeControl() = 0;
	virtual void ReleaseControl() = 0;
	virtual void Arm() = 0;
	virtual void Disarm() = 0;
	virtual void Takeoff(double altitude_m) = 0;
	virtual void Land() = 0;
	virtual void CmdPosition(double north, double east, double down, double heading_rad) = 0;
	virtual void Stop() = 0;
};
```

Implementations:

- `MavsdkVehicle` (production-ish)
- `BridgeVehicle` (talks to a simulator bridge)
- `MockVehicle` (unit tests)

### 4.6 The controller (FSM) class

The controller runs guards and transitions. For “more real-time”, run it in a fixed-rate loop.

Key responsibilities:

- Hold `FlightState`.
- Hold waypoint list + index.
- Accept latest telemetry sample.
- Issue commands only in transitions.

```cpp
class BackyardFlyerController {
 public:
	BackyardFlyerController(const Config& cfg, IVehicle& vehicle)
			: cfg_(cfg), vehicle_(vehicle), planner_(cfg) {}

	void UpdateTelemetry(const TelemetrySample& s) {
		latest_ = s;
		has_sample_ = true;
	}

	void Tick() {
		if (!has_sample_) return;
		const auto s = latest_; // copy locally (avoid holding locks)

		switch (state_) {
			case FlightState::Manual:
				if (in_mission_) ArmingTransition();
				break;

			case FlightState::Arming:
				if (s.armed) TakeoffTransition();
				break;

			case FlightState::Takeoff:
				if (AltitudeReached(s, cfg_)) {
					if (!waypoints_initialized_) {
						waypoints_ = planner_.BuildBox(s.position_ned.north, s.position_ned.east);
						waypoint_idx_ = 0;
						waypoints_initialized_ = true;
					}
					WaypointTransition();
				}
				break;

			case FlightState::Waypoint:
				if (WaypointReachedXY(s, cfg_, target_)) {
					if (waypoint_idx_ < waypoints_.size()) {
						WaypointTransition();
					} else {
						LandingTransition();
					}
				}
				break;

			case FlightState::Landing:
				if (ReadyToDisarm(s, cfg_)) DisarmingTransition();
				break;

			case FlightState::Disarming:
				if (!s.armed) ManualTransition();
				break;
		}
	}

 private:
	void ArmingTransition() {
		vehicle_.TakeControl();
		vehicle_.Arm();
		state_ = FlightState::Arming;
	}

	void TakeoffTransition() {
		vehicle_.Takeoff(cfg_.target_altitude_m);
		state_ = FlightState::Takeoff;
	}

	void WaypointTransition() {
		if (waypoint_idx_ >= waypoints_.size()) {
			LandingTransition();
			return;
		}
		target_ = waypoints_[waypoint_idx_++];
		vehicle_.CmdPosition(target_.north, target_.east, target_.down, cfg_.heading_rad);
		state_ = FlightState::Waypoint;
	}

	void LandingTransition() {
		vehicle_.Land();
		state_ = FlightState::Landing;
	}

	void DisarmingTransition() {
		vehicle_.Disarm();
		state_ = FlightState::Disarming;
	}

	void ManualTransition() {
		vehicle_.ReleaseControl();
		vehicle_.Stop();
		in_mission_ = false;
		state_ = FlightState::Manual;
	}

	const Config& cfg_;
	IVehicle& vehicle_;
	BoxPlanner planner_;

	FlightState state_{FlightState::Manual};
	bool in_mission_{true};

	TelemetrySample latest_{};
	bool has_sample_{false};

	std::array<NED, 4> waypoints_{};
	bool waypoints_initialized_{false};
	size_t waypoint_idx_{0};
	NED target_{};
};
```

Real-time note:

- `Tick()` should execute quickly and predictably.
- The vehicle commands may be asynchronous (MAVSDK) or buffered (bridge).

---

## 5) Concurrency design (telemetry thread + control loop thread)

For real-time-ish behavior, separate “I/O” from “decision making.”

### 5.1 Recommended threading

1) **Telemetry receiver thread**
	 - Receives MAVLink/MAVSDK updates.
	 - Writes latest `TelemetrySample` into a shared location.

2) **Control loop thread**
	 - Wakes up at fixed rate (e.g., 100 Hz).
	 - Reads latest sample.
	 - Calls `controller.Tick()`.

### 5.2 Data handoff options

- Simplest: `std::mutex` protecting a `TelemetrySample`.
- Better: a double-buffer with atomic index.
- Best (when needed): lock-free ring buffer.

MVP recommendation: start with `std::mutex` and keep the critical section tiny.

Pseudo:

```cpp
std::mutex m;
TelemetrySample latest;
bool has_sample = false;

// telemetry callback:
{
	std::lock_guard<std::mutex> lk(m);
	latest = sample;
	has_sample = true;
}

// control loop:
TelemetrySample local;
{
	std::lock_guard<std::mutex> lk(m);
	if (!has_sample) return;
	local = latest;
}
controller.UpdateTelemetry(local);
controller.Tick();
```

---

## 6) C++ MVP definition (what “done” looks like)

### MVP scope

- Same mission behavior as Python:
	- Arm → takeoff to 3m → fly 4 waypoints → land → disarm → stop.
- Fixed heading.
- Minimal tolerances.
- Logging to file for validation.

### MVP acceptance criteria

- Observed in simulator/SITL:
	- Achieves ~3m altitude.
	- Flies a 10m square.
	- Lands and disarms.
	- Exits cleanly.

### MVP “real-time” acceptance criteria (pragmatic)

- Control loop stable at configured rate (e.g., 100 Hz), with low jitter.
- No command spam (transitions happen once).

---

## 7) Modular C++ evolution (production-friendly layout)

Once MVP works, modularize similarly to the Python modular plan, but using CMake.

Suggested structure:

```text
backyard_flyer_cpp/
	CMakeLists.txt
	include/
		backyard/
			config.hpp
			types.hpp
			planner.hpp
			guards.hpp
			vehicle.hpp
			controller.hpp
			telemetry.hpp
	src/
		planner.cpp
		controller.cpp
		mavsdk_vehicle.cpp            # or mavlink_vehicle.cpp
		telemetry_logger.cpp
		main.cpp
	tests/
		test_planner.cpp
		test_guards.cpp
```

Engineering notes:

- Keep `include/` headers clean and stable.
- Unit test planner/guards without any MAVLink dependency.
- Treat transport (MAVSDK/MAVLink) as a plug-in.

---

## 8) Telemetry and logging (C++ version)

For validation and later MiNiFi/NiFi ingestion, log structured events:

- `state_transition`
- `command_waypoint`
- `position_sample` (throttled)
- `mission_summary`

Recommendation:

- Use JSON Lines (`.jsonl`) so MiNiFi can tail and ship.
- Keep the logging path non-blocking:
	- push events to a queue
	- write from a logger thread

---

## 9) MiNiFi C++ → NiFi evolution approach (edge-to-platform)

This mirrors the Python design’s evolution, but assumes the controller is now C++.

### 9.1 Edge (MiNiFi C++)

Run on the vehicle/edge computer:

- The C++ controller writes JSONL telemetry files (or a local UDP stream).
- MiNiFi C++ tails those files / listens to the local stream.
- MiNiFi buffers and forwards to central NiFi.

Why MiNiFi C++ fits:

- Lower footprint for edge.
- Good for intermittent connectivity + backpressure.

### 9.2 Platform (NiFi)

NiFi responsibilities:

- Ingest telemetry from many vehicles.
- Validate schema.
- Enrich with metadata.
- Route to storage (object storage, TSDB, lake/warehouse).
- Alert on mission failures or anomalous patterns.

Critical boundary:

- NiFi must not become an actuation dependency.

### 9.3 Evolution roadmap

1) **C++ MVP:** controller + local logs.
2) **Modular C++:** clean interfaces + unit tests.
3) **Edge shipping:** MiNiFi C++ tail/ship logs with buffering.
4) **Platform:** NiFi routes + stores + powers analytics.

---

## 10) Validation strategy for the transition

### 10.1 Functional parity checks

- Waypoint coordinates match the Python planner for the same origin.
- State transition ordering matches.
- Same tolerances.

### 10.2 Performance checks

- Control loop rate achieved (log loop period/jitter).
- Telemetry thread never blocks on controller logic.

### 10.3 Safety checks

- No repeated arming/takeoff/land commands (idempotence).
- Disarm only when near ground and low horizontal velocity.

---

## 11) What not to over-engineer (yet)

To keep the C++ transition tractable:

- Don’t add advanced planners.
- Don’t add obstacle avoidance.
- Don’t add distributed control.

Prove correctness + stability first, then evolve.

