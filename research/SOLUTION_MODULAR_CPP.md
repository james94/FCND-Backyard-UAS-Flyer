# Backyard Flyer — Modular, Class-Based C++ Solution (Follow-Along Guide)

This document transitions the modular Python design into a **modular, class-based C++ design** optimized for **more real-time performance**.

It is intentionally written as a “type-it-yourself” guide: you can create these files in a fresh location, type the code manually, compile, run unit tests, then integrate with a simulator transport.

Source alignment:

- Mission + API concepts: `README.md`
- Python architecture baseline: `SOLUTION_DESIGN_PY.md` and `SOLUTION_DESIGN_FOLDER_STRUCTURE_PY.md`
- C++ real-time model: `SOLUTION_DESIGN_CPP.md`
- C++ repo layout: `SOLUTION_DESIGN_FOLDER_STRUCTURE_CPP.md`

Mission reminder:

- Fly a **10m box** at **3m altitude**, then land/disarm/end mission.
- Local frame is **NED**; altitude “up” corresponds to `down = -3.0` **in telemetry**.
	- If your vehicle command API expects altitude-up (like UdaciDrone `cmd_position`), translate in the adapter layer.

README note about “manual flight first”:

- Manual control is something you do in the simulator UI (or via a separate ground-station tool).
- The C++ code in this guide focuses on the **autonomous control state machine**.

---

## 0) Target end state (what you’ll have)

You will end up with:

1) A small C++ core library that contains **only** deterministic flight logic:
	 - `Config`
	 - `NED` / `TelemetrySample`
	 - `BoxPlanner`
	 - `Guards` (pure functions)
	 - `BackyardFlyerController` (FSM + transitions)
	 - `IVehicle` interface (adapter seam)

2) A C++ app that wires:
	 - telemetry receiver thread (transport-specific)
	 - fixed-rate control loop thread (calls `Tick()`)

3) Unit tests using a `MockVehicle` (so you can validate logic without MAVLink/MAVSDK).

This mirrors the Python modular guide idea:

- Python “adapter” → C++ `IVehicle`
- Python FSM → C++ `BackyardFlyerController`
- Python guards/planner → C++ `guards.hpp` / `planner.*`

---

## 1) Recommended folder layout (matches the C++ folder-structure doc)

Create this structure (Stage 1 + Stage 2 optional):

```text
FCND-Backyard-UAS-Flyer/
	cpp/
		backyard_flyer_core/
			CMakeLists.txt
			include/
				backyard/
					config.hpp
					types.hpp
					telemetry.hpp
					planner.hpp
					guards.hpp
					vehicle.hpp
					controller.hpp
			src/
				planner.cpp
				controller.cpp
			tests/
				test_planner.cpp
				test_guards.cpp
				test_controller.cpp
		backyard_flyer_app/
			CMakeLists.txt
			src/
				main.cpp
				# optional integration modules (write later)
				bridge_vehicle.cpp
				mavsdk_vehicle.cpp
```

If you want a “single build”, add a root `cpp/CMakeLists.txt` that includes both subdirs. (This is optional for the guide; you can build each part separately.)

---

## 2) Step-by-step build-up plan (how to follow along)

To keep it learnable and testable:

1) Implement pure data + config (`types.hpp`, `config.hpp`, `telemetry.hpp`).
2) Implement planner (`planner.hpp/.cpp`) and unit test it.
3) Implement guards (`guards.hpp`) and unit test them.
4) Implement `IVehicle` (`vehicle.hpp`) and a `MockVehicle` in tests.
5) Implement controller (`controller.hpp/.cpp`) and unit test transitions.
6) Implement app wiring (`main.cpp`) with a placeholder “telemetry source”.
7) Swap placeholder telemetry source for a bridge or MAVSDK implementation.

You’ll know you’re doing it “right” when:

- The tests pass without any simulator.
- The controller never spams commands (commands only happen on transitions).
- The control loop runs at a stable rate (e.g., 100 Hz).

---

## 3) Core types (small structs, no hot-path allocations)

### 3.1 `include/backyard/types.hpp`

Purpose: explicit coordinate types + small helpers.

```cpp
#pragma once

#include <cstdint>

namespace backyard {

struct NED {
	double north{0.0};
	double east{0.0};
	double down{0.0};
};

inline NED operator+(const NED& a, const NED& b) {
	return NED{a.north + b.north, a.east + b.east, a.down + b.down};
}

inline NED operator-(const NED& a, const NED& b) {
	return NED{a.north - b.north, a.east - b.east, a.down - b.down};
}

}  // namespace backyard
```

Design notes:

- Keep this header tiny. It will be included everywhere.
- `NED` matches the Udacity local frame from `README.md`.

---

### 3.2 `include/backyard/config.hpp`

Purpose: keep mission parameters in one place (mirrors the Python `MissionConfig`).

```cpp
#pragma once

namespace backyard {

struct Config {
	double box_size_m{10.0};
	double target_altitude_m{3.0};

	double pos_tol_m{0.5};
	double vel_xy_tol_mps{0.5};
	double ground_down_tol_m{0.1};
	double takeoff_ratio{0.95};

	double heading_rad{0.0};

	// “More real-time”: controller processing cadence
	double control_rate_hz{100.0};
};

}  // namespace backyard
```

---

### 3.3 `include/backyard/telemetry.hpp`

Purpose: standardize the telemetry sample the controller consumes.

```cpp
#pragma once

#include <cstdint>

#include "backyard/types.hpp"

namespace backyard {

struct TelemetrySample {
	NED position_ned;
	NED velocity_ned;
	bool armed{false};
	bool guided{false};
	uint64_t t_us{0};
};

}  // namespace backyard
```

Notes:

- Keep it POD-like.
- Timestamp can be “best available”; monotonic microseconds is fine.

---

## 4) Planner (10m box waypoints)

### 4.1 `include/backyard/planner.hpp`

Purpose: generate the 4 corners of the box in local coordinates.

```cpp
#pragma once

#include <array>

#include "backyard/config.hpp"
#include "backyard/types.hpp"

namespace backyard {

class BoxPlanner {
 public:
	explicit BoxPlanner(const Config& cfg) : cfg_(cfg) {}

	// Origin is current local (north,east). Altitude uses NED down sign.
	std::array<NED, 4> BuildBox(double n0, double e0) const;

 private:
	const Config& cfg_;
};

}  // namespace backyard
```

### 4.2 `src/planner.cpp`

```cpp
#include "backyard/planner.hpp"

namespace backyard {

std::array<NED, 4> BoxPlanner::BuildBox(double n0, double e0) const {
	const double d = -cfg_.target_altitude_m;
	const double s = cfg_.box_size_m;

	return {
			NED{n0 + s, e0 + 0.0, d},
			NED{n0 + s, e0 + s, d},
			NED{n0 + 0.0, e0 + s, d},
			NED{n0 + 0.0, e0 + 0.0, d},
	};
}

}  // namespace backyard
```

---

## 5) Guards (pure functions)

### 5.1 `include/backyard/guards.hpp`

Purpose: C++ equivalent of Python `guards.py`.

```cpp
#pragma once

#include <cmath>

#include "backyard/config.hpp"
#include "backyard/telemetry.hpp"
#include "backyard/types.hpp"

namespace backyard::guards {

inline double Norm2D(double x, double y) {
	return std::sqrt(x * x + y * y);
}

inline bool AltitudeReached(const TelemetrySample& s, const Config& cfg) {
	const double altitude_m = -s.position_ned.down;  // NED: down is negative when up
	return altitude_m >= cfg.takeoff_ratio * cfg.target_altitude_m;
}

inline bool WaypointReachedXY(const TelemetrySample& s, const Config& cfg, const NED& target) {
	const double dn = s.position_ned.north - target.north;
	const double de = s.position_ned.east - target.east;
	return Norm2D(dn, de) <= cfg.pos_tol_m;
}

inline bool ReadyToDisarm(const TelemetrySample& s, const Config& cfg) {
	const bool near_ground = std::abs(s.position_ned.down) <= cfg.ground_down_tol_m;
	const double vxy = Norm2D(s.velocity_ned.north, s.velocity_ned.east);
	const bool stopped = vxy <= cfg.vel_xy_tol_mps;
	return near_ground && stopped;
}

}  // namespace backyard::guards
```

---

## 6) Vehicle interface (adapter seam)

In Python, UdaciDrone is your “vehicle API”. In C++, you want a seam so that:

- The controller can be unit tested without a simulator.
- Transport/MAVLink/MAVSDK code stays out of your FSM core.

### 6.1 `include/backyard/vehicle.hpp`

```cpp
#pragma once

#include "backyard/types.hpp"

namespace backyard {

class IVehicle {
 public:
	virtual ~IVehicle() = default;

	virtual void TakeControl() = 0;
	virtual void ReleaseControl() = 0;

	virtual void Arm() = 0;
	virtual void Disarm() = 0;

	virtual void Takeoff(double target_altitude_m) = 0;
	virtual void Land() = 0;

	virtual void CmdPosition(const NED& ned, double heading_rad) = 0;

	// Optional lifecycle hooks (depends on your transport)
	virtual void Stop() = 0;
};

}  // namespace backyard
```

Notes:

- This matches the outgoing command list in `README.md` at the concept level.
- Keep it minimal; you can extend later for “set home”, etc.

---

## 7) Controller / FSM (transitions only send commands once)

We’ll implement the same states:

- Manual → Arming → Takeoff → Waypoint → Landing → Disarming → Manual

### 7.1 `include/backyard/controller.hpp`

```cpp
#pragma once

#include <array>
#include <cstddef>
#include <optional>

#include "backyard/config.hpp"
#include "backyard/planner.hpp"
#include "backyard/telemetry.hpp"
#include "backyard/types.hpp"
#include "backyard/vehicle.hpp"

namespace backyard {

enum class FlightState {
	Manual,
	Arming,
	Takeoff,
	Waypoint,
	Landing,
	Disarming,
};

class BackyardFlyerController {
 public:
	BackyardFlyerController(const Config& cfg, IVehicle& vehicle);

	void StartMission();
	void UpdateTelemetry(const TelemetrySample& sample);
	void Tick();

	FlightState state() const { return state_; }
	bool in_mission() const { return in_mission_; }

 private:
	void SetState(FlightState s);

	// Transitions (actions)
	void ArmingTransition();
	void TakeoffTransition();
	void WaypointTransition();
	void LandingTransition();
	void DisarmingTransition();
	void ManualTransition();

	const Config& cfg_;
	IVehicle& vehicle_;
	BoxPlanner planner_;

	FlightState state_{FlightState::Manual};
	bool in_mission_{false};

	std::optional<TelemetrySample> last_sample_;

	std::array<NED, 4> waypoints_{};
	std::size_t waypoint_index_{0};
	bool has_plan_{false};
	NED target_{};
};

}  // namespace backyard
```

Why `Tick()` + `UpdateTelemetry()`?

- `UpdateTelemetry()` is called by the control loop after it copies the latest sample.
- `Tick()` is where you evaluate guards and do transitions.

This matches the “telemetry thread writes, fixed-rate loop reads” model from `SOLUTION_DESIGN_CPP.md`.

### 7.2 `src/controller.cpp`

```cpp
#include "backyard/controller.hpp"

#include "backyard/guards.hpp"

namespace backyard {

BackyardFlyerController::BackyardFlyerController(const Config& cfg, IVehicle& vehicle)
		: cfg_(cfg), vehicle_(vehicle), planner_(cfg) {}

void BackyardFlyerController::StartMission() {
	in_mission_ = true;
	// We keep the state machine event-driven in spirit: Tick() will advance states
	// once the appropriate telemetry conditions are observed.
}

void BackyardFlyerController::UpdateTelemetry(const TelemetrySample& sample) {
	last_sample_ = sample;
}

void BackyardFlyerController::Tick() {
	if (!in_mission_) {
		return;
	}
	if (!last_sample_.has_value()) {
		return;
	}

	const auto& s = *last_sample_;

	switch (state_) {
		case FlightState::Manual:
			ArmingTransition();
			break;

		case FlightState::Arming:
			// Equivalent of Python state_callback guard
			if (s.armed) {
				TakeoffTransition();
			}
			break;

		case FlightState::Takeoff:
			if (guards::AltitudeReached(s, cfg_)) {
				WaypointTransition();
			}
			break;

		case FlightState::Waypoint:
			if (guards::WaypointReachedXY(s, cfg_, target_)) {
				if (waypoint_index_ < waypoints_.size()) {
					WaypointTransition();
				} else {
					LandingTransition();
				}
			}
			break;

		case FlightState::Landing:
			if (guards::ReadyToDisarm(s, cfg_)) {
				DisarmingTransition();
			}
			break;

		case FlightState::Disarming:
			if (!s.armed) {
				ManualTransition();
			}
			break;
	}
}

void BackyardFlyerController::SetState(FlightState s) {
	state_ = s;
}

void BackyardFlyerController::ArmingTransition() {
	vehicle_.TakeControl();
	vehicle_.Arm();
	SetState(FlightState::Arming);
}

void BackyardFlyerController::TakeoffTransition() {
	vehicle_.Takeoff(cfg_.target_altitude_m);
	SetState(FlightState::Takeoff);
}

void BackyardFlyerController::WaypointTransition() {
	if (!last_sample_.has_value()) {
		return;
	}

	if (!has_plan_) {
		const auto& s = *last_sample_;
		waypoints_ = planner_.BuildBox(s.position_ned.north, s.position_ned.east);
		waypoint_index_ = 0;
		has_plan_ = true;
	}

	if (waypoint_index_ >= waypoints_.size()) {
		LandingTransition();
		return;
	}

	target_ = waypoints_[waypoint_index_];
	waypoint_index_++;

	vehicle_.CmdPosition(target_, cfg_.heading_rad);
	SetState(FlightState::Waypoint);
}

void BackyardFlyerController::LandingTransition() {
	vehicle_.Land();
	SetState(FlightState::Landing);
}

void BackyardFlyerController::DisarmingTransition() {
	vehicle_.Disarm();
	SetState(FlightState::Disarming);
}

void BackyardFlyerController::ManualTransition() {
	vehicle_.ReleaseControl();
	vehicle_.Stop();
	in_mission_ = false;
	SetState(FlightState::Manual);
}

}  // namespace backyard
```

Important notes:

- This controller never sends commands “every tick”; only transitions send commands.
- The `WaypointTransition()` computes waypoints once, then commands one waypoint per transition.

---

## 8) Unit tests (validate behavior before transport integration)

The tests below are written so you can validate logic without any simulator.

Test framework choice:

- Keep it simple: use a single-header framework like doctest.
- Or use Catch2.

This guide uses doctest-style macros in examples, but you can adapt to any.

### 8.1 `cpp/backyard_flyer_core/CMakeLists.txt` (minimal)

```cmake
cmake_minimum_required(VERSION 3.16)
project(backyard_flyer_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(backyard_flyer_core
	src/planner.cpp
	src/controller.cpp
)

target_include_directories(backyard_flyer_core PUBLIC include)

option(BY_BUILD_TESTS "Build tests" ON)
if(BY_BUILD_TESTS)
	enable_testing()

	# Option A: bring your own doctest header in tests/third_party/doctest/doctest.h
	add_executable(test_planner tests/test_planner.cpp)
	target_link_libraries(test_planner PRIVATE backyard_flyer_core)
	add_test(NAME test_planner COMMAND test_planner)

	add_executable(test_guards tests/test_guards.cpp)
	target_link_libraries(test_guards PRIVATE backyard_flyer_core)
	add_test(NAME test_guards COMMAND test_guards)

	add_executable(test_controller tests/test_controller.cpp)
	target_link_libraries(test_controller PRIVATE backyard_flyer_core)
	add_test(NAME test_controller COMMAND test_controller)
endif()
```

### 8.1.1 Build + run tests (commands)

From `cpp/backyard_flyer_core/`:

```sh
cmake -S . -B build -DBY_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

Suggested workflow while typing this guide:

- After finishing `types.hpp`, `config.hpp`, `telemetry.hpp`: run a build to catch missing includes.
- After finishing `planner.*`: run `test_planner`.
- After finishing `guards.hpp`: run `test_guards`.
- After finishing `controller.*`: run `test_controller`.

### 8.2 `tests/test_planner.cpp`

```cpp
// Replace with your chosen test framework include.
// #include "doctest/doctest.h"

#include "backyard/config.hpp"
#include "backyard/planner.hpp"

// TEST_CASE("BuildBox returns 4 waypoints") {
//   backyard::Config cfg;
//   cfg.box_size_m = 10.0;
//   cfg.target_altitude_m = 3.0;
//
//   backyard::BoxPlanner planner(cfg);
//   auto wps = planner.BuildBox(0.0, 0.0);
//
//   CHECK(wps.size() == 4);
//   CHECK(wps[0].north == 10.0);
//   CHECK(wps[0].east == 0.0);
//   CHECK(wps[0].down == -3.0);
// }

int main() {
	return 0;
}
```

Tip: if you don’t want to wire a framework right now, you can start with `assert(...)` and graduate later.

### 8.3 `tests/test_guards.cpp`

```cpp
#include <cassert>

#include "backyard/config.hpp"
#include "backyard/guards.hpp"
#include "backyard/telemetry.hpp"

int main() {
	backyard::Config cfg;
	cfg.target_altitude_m = 3.0;
	cfg.takeoff_ratio = 0.95;
	cfg.pos_tol_m = 0.5;

	backyard::TelemetrySample s;
	s.position_ned = backyard::NED{0.0, 0.0, -3.0};
	assert(backyard::guards::AltitudeReached(s, cfg));

	backyard::NED target{10.0, 0.0, -3.0};
	s.position_ned = backyard::NED{10.2, 0.1, -3.0};
	assert(backyard::guards::WaypointReachedXY(s, cfg, target));

	return 0;
}
```

### 8.4 `tests/test_controller.cpp` (with `MockVehicle`)

```cpp
#include <cassert>
#include <vector>

#include "backyard/config.hpp"
#include "backyard/controller.hpp"
#include "backyard/vehicle.hpp"

struct Call {
	enum class Type { TakeControl, ReleaseControl, Arm, Disarm, Takeoff, Land, CmdPosition, Stop } type;
	backyard::NED ned;
	double heading{0.0};
	double alt{0.0};
};

class MockVehicle : public backyard::IVehicle {
 public:
	std::vector<Call> calls;

	void TakeControl() override { calls.push_back({Call::Type::TakeControl}); }
	void ReleaseControl() override { calls.push_back({Call::Type::ReleaseControl}); }
	void Arm() override { calls.push_back({Call::Type::Arm}); }
	void Disarm() override { calls.push_back({Call::Type::Disarm}); }
	void Takeoff(double target_altitude_m) override { calls.push_back({Call::Type::Takeoff, {}, 0.0, target_altitude_m}); }
	void Land() override { calls.push_back({Call::Type::Land}); }
	void CmdPosition(const backyard::NED& ned, double heading_rad) override {
		Call c{Call::Type::CmdPosition};
		c.ned = ned;
		c.heading = heading_rad;
		calls.push_back(c);
	}
	void Stop() override { calls.push_back({Call::Type::Stop}); }
};

int main() {
	backyard::Config cfg;
	MockVehicle vehicle;
	backyard::BackyardFlyerController ctl(cfg, vehicle);

	ctl.StartMission();

	backyard::TelemetrySample s;
	s.armed = false;
	ctl.UpdateTelemetry(s);
	ctl.Tick();
	assert(vehicle.calls.size() >= 2);  // TakeControl + Arm

	// Simulate armed
	s.armed = true;
	ctl.UpdateTelemetry(s);
	ctl.Tick();
	// Expect takeoff command
	bool saw_takeoff = false;
	for (const auto& c : vehicle.calls) {
		if (c.type == Call::Type::Takeoff) {
			saw_takeoff = true;
			break;
		}
	}
	assert(saw_takeoff);

	return 0;
}
```

This test only checks the first few transitions; you can extend it to cover waypoint command count (4) and landing/disarm.

---

## 9) App wiring (fixed-rate loop + telemetry handoff)

The core controller is transport-agnostic. The app is where you integrate a transport (bridge, MAVSDK, MAVLink).

### 9.0 `cpp/backyard_flyer_app/CMakeLists.txt` (minimal)

This expects you build the core first, or that you have a “superbuild” that adds both.

If you want to build it standalone, you can add the core as a subdirectory via a relative path.

```cmake
cmake_minimum_required(VERSION 3.16)
project(backyard_flyer_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# For a simple follow-along approach, include the core directly:
add_subdirectory(../backyard_flyer_core backyard_flyer_core_build)

add_executable(backyard_flyer_app
	src/main.cpp
)

target_link_libraries(backyard_flyer_app PRIVATE backyard_flyer_core)
```

### 9.1 `cpp/backyard_flyer_app/src/main.cpp` (skeleton)

This shows the recommended Model B: telemetry thread updates the “latest sample”, control thread runs `Tick()` at fixed rate.

```cpp
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "backyard/config.hpp"
#include "backyard/controller.hpp"
#include "backyard/telemetry.hpp"
#include "backyard/vehicle.hpp"

// TODO: replace this with BridgeVehicle or MavsdkVehicle
class DummyVehicle : public backyard::IVehicle {
 public:
	void TakeControl() override {}
	void ReleaseControl() override {}
	void Arm() override {}
	void Disarm() override {}
	void Takeoff(double) override {}
	void Land() override {}
	void CmdPosition(const backyard::NED&, double) override {}
	void Stop() override {}
};

int main() {
	backyard::Config cfg;
	DummyVehicle vehicle;
	backyard::BackyardFlyerController controller(cfg, vehicle);

	std::mutex m;
	backyard::TelemetrySample latest;
	bool has_sample = false;
	std::atomic<bool> running{true};

	// Telemetry receiver thread (placeholder)
	std::thread rx([&]() {
		while (running.load()) {
			backyard::TelemetrySample s;
			// TODO: fill s from your transport
			{
				std::lock_guard<std::mutex> lk(m);
				latest = s;
				has_sample = true;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	});

	controller.StartMission();

	const auto period = std::chrono::duration<double>(1.0 / cfg.control_rate_hz);
	auto next = std::chrono::steady_clock::now();

	while (controller.in_mission()) {
		next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

		backyard::TelemetrySample local;
		bool ok = false;
		{
			std::lock_guard<std::mutex> lk(m);
			ok = has_sample;
			local = latest;
		}
		if (ok) {
			controller.UpdateTelemetry(local);
			controller.Tick();
		}

		std::this_thread::sleep_until(next);
	}

	running.store(false);
	rx.join();
	return 0;
}

### 9.1.1 `main.cpp` wiring for the UdaciDrone bridge (BridgeVehicle)

Once you implement `BridgeTransport` + `BridgeVehicle` (Section 12.4), the placeholder receiver thread becomes:

- a thread inside `BridgeTransport` that reads telemetry JSONL from Python
- a `BridgeVehicle` that sends command JSONL to Python

Minimal wiring sketch:

```cpp
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "backyard/config.hpp"
#include "backyard/controller.hpp"
#include "backyard/telemetry.hpp"

#include "bridge_transport.hpp"
#include "bridge_vehicle.hpp"

int main() {
	backyard::Config cfg;

	std::mutex m;
	backyard::TelemetrySample latest;
	bool has_sample = false;

	BridgeTransport transport;
	transport.Connect("127.0.0.1", 9002);
	transport.StartTelemetryThread([&](const backyard::TelemetrySample& s) {
		std::lock_guard<std::mutex> lk(m);
		latest = s;
		has_sample = true;
	});

	BridgeVehicle vehicle(transport);
	backyard::BackyardFlyerController controller(cfg, vehicle);
	controller.StartMission();

	const auto period = std::chrono::duration<double>(1.0 / cfg.control_rate_hz);
	auto next = std::chrono::steady_clock::now();

	while (controller.in_mission()) {
		next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

		backyard::TelemetrySample local;
		bool ok = false;
		{
			std::lock_guard<std::mutex> lk(m);
			ok = has_sample;
			local = latest;
		}
		if (ok) {
			controller.UpdateTelemetry(local);
			controller.Tick();
		}
		std::this_thread::sleep_until(next);
	}

	// Ask the Python bridge to shut down cleanly.
	vehicle.Stop();
	transport.Close();
	return 0;
}
```
```

### 9.2 Build + run the app (commands)

From `cpp/backyard_flyer_app/`:

```sh
cmake -S . -B build
cmake --build build
./build/backyard_flyer_app
```

Key “real-time-ish” choices:

- Controller logic runs at fixed cadence (`sleep_until`).
- Transport thread does minimal work.
- Shared state is protected by a small mutex.

---

## 10) How to validate incrementally (practical checklist)

### 10.1 Fast validation (no simulator)

- Compile core + run unit tests.
- Confirm:
	- `BuildBox()` generates the correct four corners.
	- NED sign is correct in telemetry/internal math (`down = -3.0` corresponds to 3m altitude).
	- Controller produces exactly 4 waypoint commands, then `Land()`, then `Disarm()`, then `ReleaseControl()` + `Stop()`.

### 10.2 Integration validation (sim / SITL)

Once you implement a real `IVehicle` adapter:

- Confirm transition ordering matches README.
- Confirm you don’t command waypoints repeatedly every tick.
- Confirm the drone flies the 10m square at ~3m altitude and ends mission.

---

## 11) Next: implementing a real `IVehicle`

This guide keeps transport code out of the core on purpose.

You can now implement either:

1) BridgeVehicle: quickest if you must keep the Unity sim tied to Python/UdaciDrone.
2) MavsdkVehicle: best if you are targeting SITL/real PX4 and want a production-friendly C++ stack.

The controller core you wrote above should not change.

---

## 12) Unity simulator integration via Python `udacidrone` (practical bridge)

This guide intentionally kept transport out of the C++ core.

However, if your immediate goal is:

- “Run the **C++ controller** against the Udacity **Unity simulator**”

…then you need a bridge, because `udacidrone` is Python-first.

### 12.1 High-level design

Run two processes:

1) **Python bridge** (uses `udacidrone`): connects to Unity MAVLink and exposes a small TCP server.
2) **C++ app**: runs your fixed-rate loop and uses a `BridgeVehicle` adapter to send commands over TCP.

Key boundary rule:

- Your C++ core uses NED (`down_m`).
- UdaciDrone `Drone.cmd_position(north, east, altitude, heading)` uses **altitude-up**.
- Therefore the adapter/bridge must translate: `altitude_m = -down_m`.

### 12.2 Minimal JSONL protocol

Use JSON Lines (one JSON object per line) over a single TCP socket.

C++ → Python (commands):

```json
{"type":"cmd","name":"take_control"}
{"type":"cmd","name":"arm"}
{"type":"cmd","name":"takeoff","altitude_m":3.0}
{"type":"cmd","name":"cmd_position","north_m":10.0,"east_m":0.0,"down_m":-3.0,"heading_rad":0.0}
{"type":"cmd","name":"land"}
{"type":"cmd","name":"disarm"}
{"type":"cmd","name":"release_control"}
{"type":"cmd","name":"stop"}
```

Python → C++ (telemetry snapshots):

```json
{"type":"telemetry","t_us":1700000001123456,
 "armed":true,"guided":true,
 "local_position_ned_m":[1.2,0.3,-3.0],
 "local_velocity_ned_mps":[0.1,0.0,0.0]}
```

This is intentionally minimal; you can extend later (global position, status, etc.).

### 12.3 Python bridge sketch (UdaciDrone + threaded connection)

This is a sketch you can put into `bridge/python/udacidrone_bridge_server.py`.

Notes:

- Use `threaded=True` so `conn.start()` does not block.
- Stream telemetry at a bounded rate (e.g., 20–50 Hz).
- Apply the `down_m → altitude_m` conversion only at the `Drone.cmd_position(...)` call.

```python
from __future__ import annotations

import json
import socket
import threading
import time

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection


def jsonl_send(sock: socket.socket, obj: dict) -> None:
	data = (json.dumps(obj) + "\n").encode("utf-8")
	sock.sendall(data)


def main() -> None:
	# Unity sim default: tcp:127.0.0.1:5760
	conn = MavlinkConnection("tcp:127.0.0.1:5760", threaded=True, PX4=False)
	drone = Drone(conn)

	# Start MAVLink receive loop in the background.
	conn.start()
	# Give it a moment to populate telemetry.
	time.sleep(1.0)

	server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	server.bind(("127.0.0.1", 9002))
	server.listen(1)
	print("Bridge listening on 127.0.0.1:9002")

	client, addr = server.accept()
	print("Client connected:", addr)
	client.settimeout(0.1)

	running = True

	def telemetry_loop() -> None:
		while running:
			lp = drone.local_position.tolist()
			lv = drone.local_velocity.tolist()
			jsonl_send(
				client,
				{
					"type": "telemetry",
					"t_us": int(time.time() * 1e6),
					"armed": bool(drone.armed),
					"guided": bool(drone.guided),
					"local_position_ned_m": lp,
					"local_velocity_ned_mps": lv,
				},
			)
			time.sleep(0.02)  # 50 Hz

	threading.Thread(target=telemetry_loop, daemon=True).start()

	buf = b""
	try:
		while True:
			try:
				chunk = client.recv(4096)
			except socket.timeout:
				continue
			if not chunk:
				break
			buf += chunk
			while b"\n" in buf:
				line, buf = buf.split(b"\n", 1)
				line = line.strip()
				if not line:
					continue
				cmd = json.loads(line.decode("utf-8"))
				if cmd.get("type") != "cmd":
					continue

				name = cmd.get("name")
				if name == "take_control":
					drone.take_control()
				elif name == "release_control":
					drone.release_control()
				elif name == "arm":
					drone.arm()
				elif name == "disarm":
					drone.disarm()
				elif name == "takeoff":
					drone.takeoff(float(cmd["altitude_m"]))
				elif name == "land":
					drone.land()
				elif name == "cmd_position":
					n = float(cmd["north_m"])
					e = float(cmd["east_m"])
					d = float(cmd["down_m"])  # NED
					heading = float(cmd.get("heading_rad", 0.0))
					alt = -d  # IMPORTANT: UdaciDrone cmd_position uses altitude-up
					drone.cmd_position(n, e, alt, heading)
				elif name == "stop":
					drone.stop()
					return
	finally:
		running = False
		try:
			client.close()
		finally:
			server.close()
			drone.stop()


if __name__ == "__main__":
	main()
```

### 12.4 C++ adapter mapping (BridgeVehicle)

Your `IVehicle` stays NED-oriented:

- `CmdPosition(const NED& ned, heading)` uses `ned.down`.

BridgeVehicle mapping to the wire protocol:

- `Takeoff(target_altitude_m)` → send `{name:"takeoff", altitude_m: target_altitude_m}`
- `CmdPosition(ned, heading)` → send `{name:"cmd_position", north_m: ned.north, east_m: ned.east, down_m: ned.down, heading_rad: heading}`

The Python side performs the only required translation for UdaciDrone:

- `altitude_m = -down_m` when calling `Drone.cmd_position(...)`.

### 12.4.1 C++ implementation: `BridgeTransport` + `BridgeVehicle`

Below is a minimal Linux/POSIX implementation that:

- opens a TCP connection to the Python bridge (default `127.0.0.1:9002`)
- sends command JSONL lines
- reads telemetry JSONL lines on a background thread and converts them to `backyard::TelemetrySample`

This intentionally uses a small JSON library to keep parsing safe.

#### Add a JSON header (single file)

Vendor `nlohmann/json.hpp` under:

`cpp/backyard_flyer_app/third_party/nlohmann/json.hpp`

You can download the single-header release from the nlohmann/json project.

#### `cpp/backyard_flyer_app/src/bridge_transport.hpp`

```cpp
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "backyard/telemetry.hpp"

class BridgeTransport {
 public:
	BridgeTransport() = default;
	~BridgeTransport();

	BridgeTransport(const BridgeTransport&) = delete;
	BridgeTransport& operator=(const BridgeTransport&) = delete;

	void Connect(const std::string& host, int port);
	void Close();

	void SendLine(const std::string& line);
	void StartTelemetryThread(std::function<void(const backyard::TelemetrySample&)> on_sample);

 private:
	std::string ReadLineBlocking();

	int fd_{-1};
	std::mutex send_mu_;

	std::atomic<bool> running_{false};
	std::thread rx_;
};
```

#### `cpp/backyard_flyer_app/src/bridge_transport.cpp`

```cpp
#include "bridge_transport.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include "third_party/nlohmann/json.hpp"

using nlohmann::json;

BridgeTransport::~BridgeTransport() {
	Close();
}

void BridgeTransport::Connect(const std::string& host, int port) {
	if (fd_ != -1) {
		throw std::runtime_error("BridgeTransport already connected");
	}

	fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
	if (fd_ < 0) {
		throw std::runtime_error(std::string("socket failed: ") + std::strerror(errno));
	}

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(static_cast<uint16_t>(port));
	if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
		::close(fd_);
		fd_ = -1;
		throw std::runtime_error("inet_pton failed for host: " + host);
	}

	if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
		::close(fd_);
		fd_ = -1;
		throw std::runtime_error(std::string("connect failed: ") + std::strerror(errno));
	}
}

void BridgeTransport::Close() {
	running_.store(false);
	if (rx_.joinable()) {
		rx_.join();
	}
	if (fd_ != -1) {
		::shutdown(fd_, SHUT_RDWR);
		::close(fd_);
		fd_ = -1;
	}
}

void BridgeTransport::SendLine(const std::string& line) {
	if (fd_ == -1) {
		throw std::runtime_error("SendLine called while not connected");
	}

	std::lock_guard<std::mutex> lk(send_mu_);
	const std::string data = line + "\n";
	const char* p = data.data();
	size_t remaining = data.size();
	while (remaining > 0) {
		const ssize_t n = ::send(fd_, p, remaining, 0);
		if (n < 0) {
			throw std::runtime_error(std::string("send failed: ") + std::strerror(errno));
		}
		p += static_cast<size_t>(n);
		remaining -= static_cast<size_t>(n);
	}
}

std::string BridgeTransport::ReadLineBlocking() {
	if (fd_ == -1) {
		throw std::runtime_error("ReadLineBlocking called while not connected");
	}

	std::string out;
	out.reserve(256);
	while (true) {
		char c = 0;
		const ssize_t n = ::recv(fd_, &c, 1, 0);
		if (n == 0) {
			throw std::runtime_error("bridge socket closed");
		}
		if (n < 0) {
			if (errno == EINTR) continue;
			throw std::runtime_error(std::string("recv failed: ") + std::strerror(errno));
		}
		if (c == '\n') {
			return out;
		}
		out.push_back(c);
	}
}

void BridgeTransport::StartTelemetryThread(std::function<void(const backyard::TelemetrySample&)> on_sample) {
	if (running_.exchange(true)) {
		throw std::runtime_error("telemetry thread already running");
	}

	rx_ = std::thread([this, cb = std::move(on_sample)]() {
		while (running_.load()) {
			const std::string line = ReadLineBlocking();
			if (line.empty()) continue;

			json j = json::parse(line);
			if (j.value("type", "") != "telemetry") continue;

			backyard::TelemetrySample s;
			s.t_us = j.value("t_us", static_cast<uint64_t>(0));
			s.armed = j.value("armed", false);
			s.guided = j.value("guided", false);

			const auto lp = j.at("local_position_ned_m");
			s.position_ned.north = lp.at(0).get<double>();
			s.position_ned.east = lp.at(1).get<double>();
			s.position_ned.down = lp.at(2).get<double>();

			const auto lv = j.at("local_velocity_ned_mps");
			s.velocity_ned.north = lv.at(0).get<double>();
			s.velocity_ned.east = lv.at(1).get<double>();
			s.velocity_ned.down = lv.at(2).get<double>();

			cb(s);
		}
	});
}
```

#### `cpp/backyard_flyer_app/src/bridge_vehicle.hpp`

```cpp
#pragma once

#include "backyard/types.hpp"
#include "backyard/vehicle.hpp"

#include "bridge_transport.hpp"

class BridgeVehicle final : public backyard::IVehicle {
 public:
	explicit BridgeVehicle(BridgeTransport& t) : t_(t) {}

	void TakeControl() override;
	void ReleaseControl() override;
	void Arm() override;
	void Disarm() override;
	void Takeoff(double target_altitude_m) override;
	void Land() override;
	void CmdPosition(const backyard::NED& ned, double heading_rad) override;
	void Stop() override;

 private:
	BridgeTransport& t_;
};
```

#### `cpp/backyard_flyer_app/src/bridge_vehicle.cpp`

```cpp
#include "bridge_vehicle.hpp"

#include <sstream>

// Minimal JSON construction (keep it simple; bridge server is strict).
// If you prefer, you can also build JSON via nlohmann::json and dump().

static std::string cmd0(const char* name) {
	std::ostringstream os;
	os << "{\"type\":\"cmd\",\"name\":\"" << name << "\"}";
	return os.str();
}

void BridgeVehicle::TakeControl() {
	t_.SendLine(cmd0("take_control"));
}

void BridgeVehicle::ReleaseControl() {
	t_.SendLine(cmd0("release_control"));
}

void BridgeVehicle::Arm() {
	t_.SendLine(cmd0("arm"));
}

void BridgeVehicle::Disarm() {
	t_.SendLine(cmd0("disarm"));
}

void BridgeVehicle::Takeoff(double target_altitude_m) {
	std::ostringstream os;
	os << "{\"type\":\"cmd\",\"name\":\"takeoff\",\"altitude_m\":" << target_altitude_m << "}";
	t_.SendLine(os.str());
}

void BridgeVehicle::Land() {
	t_.SendLine(cmd0("land"));
}

void BridgeVehicle::CmdPosition(const backyard::NED& ned, double heading_rad) {
	std::ostringstream os;
	os << "{\"type\":\"cmd\",\"name\":\"cmd_position\""
	   << ",\"north_m\":" << ned.north
	   << ",\"east_m\":" << ned.east
	   << ",\"down_m\":" << ned.down
	   << ",\"heading_rad\":" << heading_rad
	   << "}";
	t_.SendLine(os.str());
}

void BridgeVehicle::Stop() {
	// Ask python bridge to shut down. It will close socket.
	t_.SendLine(cmd0("stop"));
}
```

Notes:

- `BridgeVehicle::CmdPosition(...)` sends `down_m` exactly as produced by your C++ core.
- The Python bridge converts `down_m` to altitude-up for UdaciDrone using `altitude_m = -down_m`.
- `Stop()` is optional but convenient for clean shutdown.

### 12.5 What this bridge gives you (and what it doesn’t)

Gives you:

- ability to regression-test your C++ core logic inside Unity without rewriting `udacidrone` in C++

Doesn’t give you (yet):

- production-grade transport, hard real-time guarantees, or multi-vehicle routing

That’s fine: this bridge is a migration tool, not the final transport.

