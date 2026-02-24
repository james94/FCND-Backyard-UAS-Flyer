
# Backyard Flyer — Proposed Repo Folder Structure (Modular C++ → ROS2 Jazzy C++ + MiNiFi C++)

This document proposes a folder structure that evolves the project from the **modular C++ approach** into a **ROS2 Jazzy C++** robotics runtime (custom nodes), while still using **MiNiFi C++** for edge telemetry shipping and ops workflows.

It is intentionally consistent with:

- `SOLUTION_DESIGN_CPP.md` (real-time framing, control-loop model)
- `SOLUTION_MODULAR_CPP.md` (core library boundary)
- `SOLUTION_DESIGN_FOLDER_STRUCTURE_CPP.md` (staged repo evolution)
- `SOLUTION_MINIFI_CPP.md` and `SOLUTION_MINIFI_CPP_V2.md` (MiNiFi boundary + JSONL contracts)

Core principle (unchanged):

- **Actuation/control stays local** (ROS2 controller + vehicle interface nodes).
- **MiNiFi/NiFi are not in the actuation path**; they handle telemetry movement and operational workflows.

---

## 1) What the structure must enable

1) Keep the deterministic flight logic as a C++ library (planner/guards/controller).
2) Build and run ROS2 Jazzy nodes that wrap/wire the logic.
3) Swap transports (Udacity simulator bridge → MAVSDK) without touching controller logic.
4) Emit JSONL telemetry/status compatible with MiNiFi.
5) Run MiNiFi edge configs beside the ROS2 system.

---

## 2) Staged evolution (recommended)

### Stage 0 — Keep Python baseline (optional regression oracle)

```text
FCND-Backyard-UAS-Flyer/
	README.md
	backyard_flyer.py
	drone.py
	research/
		SOLUTION_DESIGN_CPP.md
		SOLUTION_MODULAR_CPP.md
		SOLUTION_MINIFI_CPP.md
		SOLUTION_MINIFI_CPP_V2.md
		SOLUTION_ROS2_JAZZY_CPP.md
		SOLUTION_DESIGN_FOLDER_STRUCTURE_ROS2_MINIFI_CPP.md
	Logs/                # runtime output (gitignored)
	.gitignore
```

Why keep Python initially:

- parity validation for mission behavior and tolerances.

### Stage 1 — Modular C++ core (unit-testable, no ROS2)

```text
FCND-Backyard-UAS-Flyer/
	cpp/
		backyard_flyer_core/
			CMakeLists.txt
			include/backyard/
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
```

This remains your “truth” for flight logic.

### Stage 2 — ROS2 Jazzy workspace (custom nodes)

Add a ROS2 workspace that consumes the core library.

```text
FCND-Backyard-UAS-Flyer/
	ros2_ws/
		src/
			backyard_flyer_msgs/
				package.xml
				CMakeLists.txt
				msg/
					Ned.msg
					TelemetrySample.msg
					CommandIntent.msg
					ControllerStatus.msg
					TelemetryEvent.msg           # optional

			backyard_flyer_core_ros/
				package.xml
				CMakeLists.txt
				# This package wraps or vendors the Stage 1 core library for ament.
				# Keep it as thin as possible.

			backyard_flyer_nodes/
				package.xml
				CMakeLists.txt
				include/
					backyard_controller_node.hpp
				src/
					backyard_controller_node.cpp
					telemetry_jsonl_logger_node.cpp

			backyard_flyer_vehicle/
				package.xml
				CMakeLists.txt
				src/
					vehicle_interface_node.cpp
					transport_mavsdk.cpp          # optional
					transport_bridge.cpp          # optional

			backyard_flyer_http_gateway/
				package.xml
				CMakeLists.txt
				src/
					ros2_http_gateway_node.cpp
```

Notes:

- `backyard_flyer_core_ros/` exists because ROS2 uses `ament_cmake` and needs packages.
- You can vendor the `cpp/backyard_flyer_core` sources into this package, or build it as an external project.
- Keep ROS2 dependencies out of the core library.

### Stage 3 — Edge telemetry shipping (MiNiFi C++)

Keep MiNiFi as a sidecar, tailing the ROS2 logger’s JSONL output.

```text
FCND-Backyard-UAS-Flyer/
	edge/
		minifi-cpp/
			README.md
			conf/
				minifi.properties
				config.yml
			scripts/
				run_minifi.sh
```

Conventions:

- ROS2 logger writes JSONL under `Logs/`.
- MiNiFi tails `Logs/telemetry.jsonl` and `Logs/controller_status.jsonl`.

### Stage 4 — Platform ingestion (NiFi)

```text
FCND-Backyard-UAS-Flyer/
	platform/
		nifi/
			README.md
			flows/
				telemetry_ingest.json
			docker/
				docker-compose.yml
```

---

## 3) “Final form” combined tree

This is the combined layout supporting all stages.

```text
FCND-Backyard-UAS-Flyer/
	README.md

	# Python baseline (optional)
	backyard_flyer.py
	drone.py

	# Core logic (no ROS2)
	cpp/
		backyard_flyer_core/

	# ROS2 runtime
	ros2_ws/
		src/
			backyard_flyer_msgs/
			backyard_flyer_core_ros/
			backyard_flyer_nodes/
			backyard_flyer_vehicle/
			backyard_flyer_http_gateway/

	# Edge
	edge/
		minifi-cpp/

	# Platform
	platform/
		nifi/

	# Docs
	research/
		SOLUTION_DESIGN_CPP.md
		SOLUTION_MODULAR_CPP.md
		SOLUTION_MINIFI_CPP.md
		SOLUTION_MINIFI_CPP_V2.md
		SOLUTION_ROS2_JAZZY_CPP.md
		SOLUTION_DESIGN_FOLDER_STRUCTURE_ROS2_MINIFI_CPP.md

	Logs/
	.gitignore
```

---

## 4) How ROS2 and MiNiFi interact in this layout

### Telemetry plane

- ROS2 nodes publish telemetry/status
- ROS2 logger node writes JSONL in `Logs/`
- MiNiFi tails JSONL and ships reliably (buffering/backpressure/routing)

### Ops/control-plane

- ROS2 HTTP gateway node exposes localhost endpoints
- MiNiFi uses built-in `InvokeHTTP` (as in `SOLUTION_MINIFI_CPP_V2.md`)

This preserves the safety boundary: MiNiFi can request start/stop/config, but actuation stays within the ROS2 control stack.

---

## 5) What NOT to do

- Don’t route flight actuation commands through MiNiFi.
- Don’t put MAVSDK/MAVLink specifics into the core controller library.
- Don’t commit logs or secrets.

