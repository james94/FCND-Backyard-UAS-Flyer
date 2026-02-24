# Backyard Flyer — Proposed Repo Folder Structure (Python → C++ Real-Time, MiNiFi C++ → NiFi)

This proposes a GitHub repo layout to support a **real-time oriented C++ rewrite** of the Backyard Flyer mission.

It is based on:

- `README.md` (mission requirements + event-driven state machine constraints)
- `SOLUTION_DESIGN_PY.md` (Python architecture baseline)
- `SOLUTION_DESIGN_FOLDER_STRUCTURE.md` (Python repo evolution model)
- `SOLUTION_DESIGN_CPP.md` (C++ real-time transition architecture)

The guiding principle stays the same:

- **Real-time control is local** (C++ controller / autopilot).
- **MiNiFi/NiFi are not part of actuation**; they are for telemetry, routing, storage, and analytics.

---

## 1) What the folder structure must enable

From `README.md`, the functional outcome is:

- Fly a **10m box** at **3m altitude**, then land/disarm/end mission.

From `SOLUTION_DESIGN_CPP.md`, the non-functional target is:

- More deterministic execution (fixed-rate control loop, explicit threading, minimal allocations).

Therefore the repo should support:

1) A clean C++ build (CMake) producing a runnable controller binary.
2) Modularization (planner/guards/controller/vehicle adapter).
3) A migration-friendly “bridge” phase (if the Unity simulator is Python/UdaciDrone-centric).
4) Telemetry schemas + sample payloads (to support MiNiFi/NiFi validation).
5) Edge configs (MiNiFi C++) and platform flows (NiFi).

---

## 2) Staged evolution (recommended)

### Stage 0 — Keep Python as baseline + regression oracle

Goal: keep the existing Python version runnable so you can compare behavior/logs.

```text
FCND-Backyard-UAS-Flyer/
	README.md
	backyard_flyer.py                  # Python baseline (may remain for reference)
	drone.py
	research/
		SOLUTION_DESIGN_PY.md
		SOLUTION_DESIGN_CPP.md
		SOLUTION_DESIGN_FOLDER_STRUCTURE.md
		SOLUTION_DESIGN_FOLDER_STRUCTURE_CPP.md
	Logs/                              # runtime output (gitignored)
	.gitignore
```

Notes:

- Even if “requirements changed to C++”, keeping Python initially is a pragmatic way to validate parity.
- Treat this as a *temporary* dependency; the C++ path becomes the primary entrypoint.

---

### Stage 1 — C++ MVP “core library” + unit tests (no MAVLink dependency)

Goal: implement the planner/guards/controller exactly as described in `SOLUTION_DESIGN_CPP.md`, but test it against `MockVehicle`.

This is where you gain confidence in the **logic** (waypoints, tolerances, state transitions) without fighting comms.

```text
FCND-Backyard-UAS-Flyer/
	cpp/
		backyard_flyer_core/
			CMakeLists.txt
			include/
				backyard/
					config.hpp
					types.hpp
					planner.hpp
					guards.hpp
					vehicle.hpp               # IVehicle interface
					controller.hpp            # BackyardFlyerController
					telemetry.hpp             # event structs
			src/
				planner.cpp
				controller.cpp
			tests/
				test_planner.cpp
				test_guards.cpp
				test_controller.cpp         # uses MockVehicle
			third_party/
				# optional: vendored test framework (or fetched by CMake)
```

Why this stage matters:

- It enforces modular design early.
- You can run fast CI-like tests locally.
- It ensures the C++ rewrite is correct before integrating real comms.

---

### Stage 2 — Choose and integrate the comms layer (Bridge first, then MAVSDK)

Goal: connect the controller to a real telemetry+command transport.

You have two common paths:

#### Stage 2A (migration-friendly): Python bridge

Use when the Unity sim is easiest to talk to with the existing Python tooling.

```text
FCND-Backyard-UAS-Flyer/
	bridge/
		python/
			README.md
			bridge_server.py              # reads UdaciDrone state; exposes to C++ (TCP/UDP)
			bridge_protocol.md            # message schemas
			scripts/
				run_bridge.sh
	cpp/
		backyard_flyer_app/
			CMakeLists.txt
			src/
				main.cpp                    # wires transport + controller + fixed-rate loop
				bridge_vehicle.cpp          # implements IVehicle
				bridge_transport.cpp        # socket client
```

Benefits:

- Fastest path to prove the C++ controller can fly the box in the same simulator.
- Lets you keep UdaciDrone simulator expectations isolated to `bridge/python/`.

#### Stage 2B (production-ish): MAVSDK (or MAVLink C)

Use when targeting SITL/real autopilot integration.

```text
FCND-Backyard-UAS-Flyer/
	cpp/
		backyard_flyer_app/
			src/
				main.cpp
				mavsdk_vehicle.cpp          # implements IVehicle via MAVSDK
				mavsdk_telemetry.cpp        # subscriptions → TelemetrySample
			cmake/
				FetchMavsdk.cmake           # or system dependency instructions
```

Design rule:

- Keep MAVSDK/MAVLink-specific code out of the controller core; it stays behind `IVehicle`.

---

### Stage 3 — Telemetry-first (schemas + samples + logger)

Goal: produce structured telemetry that’s easy to ship/validate in dataflows.

```text
FCND-Backyard-UAS-Flyer/
	schemas/
		telemetry_event.schema.json
		telemetry_sample.schema.json
	data/
		samples/
			telemetry_example.jsonl
	cpp/
		backyard_flyer_core/
			include/
				backyard/
					telemetry.hpp
			src/
				telemetry_logger.cpp         # JSONL writer (prefer async)
```

Recommendations for real-time friendliness:

- Controller thread emits small “events” into a queue.
- Logger thread flushes to disk/network.

---

### Stage 4 — Edge shipping (MiNiFi C++)

Goal: ship telemetry off-device without impacting control.

```text
FCND-Backyard-UAS-Flyer/
	edge/
		minifi-cpp/
			README.md
			conf/
				minifi.properties
				config.yml                  # tail JSONL → forward
			scripts/
				run_minifi.sh
			docker/
				Dockerfile                  # optional
		telemetry/
			README.md
			example_output/
				telemetry.jsonl
```

Patterns to prefer:

- File tailing (controller writes JSONL; MiNiFi tails and forwards).
- Buffered transport (MiNiFi handles intermittent connections).

---

### Stage 5 — Platform ingestion (NiFi)

Goal: central collection, validation, enrichment, routing.

```text
FCND-Backyard-UAS-Flyer/
	platform/
		nifi/
			README.md
			flows/
				telemetry_ingest.json
			parameters/
				dev.params.json
				prod.params.json
			docker/
				docker-compose.yml
		nifi-registry/
			README.md                     # optional
	infra/
		storage/
			README.md
		monitoring/
			README.md
```

NiFi flow responsibilities align with the Python design docs:

- Ingest → Validate schema → Enrich → Route to storage → Alert.

---

## 3) Recommended “final form” repo tree (supports all stages)

This is the combined tree that supports migration + long-term evolution.

```text
FCND-Backyard-UAS-Flyer/
	README.md

	# Python baseline (optional but useful early)
	backyard_flyer.py
	drone.py

	# C++ implementation
	cpp/
		backyard_flyer_core/
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
				telemetry_logger.cpp
			tests/
				test_planner.cpp
				test_guards.cpp
				test_controller.cpp
		backyard_flyer_app/
			CMakeLists.txt
			src/
				main.cpp
				# choose one (or both) during migration:
				bridge_vehicle.cpp
				mavsdk_vehicle.cpp
				mavlink_vehicle.cpp
			scripts/
				run_app.sh

	# Migration bridge (only if needed for Unity sim)
	bridge/
		python/
			README.md
			bridge_server.py
			bridge_protocol.md
			scripts/
				run_bridge.sh

	# Telemetry schemas + sample payloads
	schemas/
		telemetry_event.schema.json
		telemetry_sample.schema.json
	data/
		samples/
			telemetry_example.jsonl

	# Edge + Platform (optional evolution)
	edge/
		minifi-cpp/
			conf/
				minifi.properties
				config.yml
			scripts/
				run_minifi.sh
	platform/
		nifi/
			flows/
				telemetry_ingest.json
			docker/
				docker-compose.yml

	research/
		SOLUTION_DESIGN_PY.md
		SOLUTION_DESIGN_CPP.md
		SOLUTION_DESIGN_FOLDER_STRUCTURE.md
		SOLUTION_DESIGN_FOLDER_STRUCTURE_CPP.md

	.gitignore
```

---

## 4) Step-by-step “Python → C++” transition using this structure

1) **Lock the mission spec** (waypoints, tolerances, state order) using the Python version as reference.
2) Implement and unit test the C++ core in `cpp/backyard_flyer_core/` using `MockVehicle`.
3) Integrate with the simulator:
	 - If Unity sim is easiest via Python: add `bridge/python/` and implement `bridge_vehicle.cpp`.
	 - Otherwise: implement `mavsdk_vehicle.cpp` (or `mavlink_vehicle.cpp`).
4) Make `cpp/backyard_flyer_app/` the primary entrypoint.
5) Add structured telemetry + schemas (`schemas/`, `telemetry_logger.cpp`).
6) Add MiNiFi C++ edge configs (`edge/minifi-cpp/`).
7) Add NiFi flows (`platform/nifi/`).

---

## 5) What NOT to include in this repo structure

To keep the project focused:

- Don’t put NiFi/MiNiFi anywhere near command/actuation.
- Don’t mix simulator-specific glue into the controller core.
- Don’t commit secrets, credentials, or runtime logs.

