# ROS2 C++ UdaciDrone Driver and Mission Stack: Software Problem Statement

## 1. Context and Motivation

The original Udacity FCND Backyard Flyer project demonstrates autonomous UAS control using a Python state machine (`MANUAL -> ARMING -> TAKEOFF -> WAYPOINT -> LANDING -> DISARMING`) over the UdaciDrone API and a Unity3D simulator.

Your target system extends this into an industry-style robotics software stack:

- ROS2 Jazzy middleware (C++) as the primary backend runtime
- Event-driven mission orchestration with SMACC2
- Optional MoveIt2 integration where motion-planning style abstractions are useful
- Ubuntu 24.04 Dockerized development and runtime for reproducible builds
- A modular architecture that can scale beyond a single FCND mission

The core challenge is to reproduce FCND mission behavior while replacing direct Python mission logic with a ROS2-native C++ architecture that still interoperates with the Unity3D simulator transport semantics used by UdaciDrone/MAVLink.

## 2. Source Analysis Summary

### 2.1 FCND Backyard Flyer Requirements (from README)

The baseline assignment requires:

- Autonomous takeoff to 3 m
- Flight through a 10 m box path
- Controlled landing and disarm
- Event-driven transitions based on telemetry callbacks

The reference behavior is callback-centric and transition criteria are evaluated from:

- `LOCAL_POSITION`
- `LOCAL_VELOCITY`
- `STATE` (armed/guided)

### 2.2 UdaciDrone API Architecture (from forked repo)

UdaciDrone has two conceptual layers:

- `Drone` abstraction: state store + command interface + callback registration
- `Connection` implementation (MAVLink): transport/protocol details

Observed key technical behavior:

- Incoming messages are dispatched by `MsgID` and update local state
- Listeners can be registered per message type
- MAVLink command surface includes arm/disarm, mode control, takeoff/land, local position and velocity control
- Simulator/PX4 behavior differences exist (example: local down sign handling in `cmd_position`)

Implication for ROS2:

- The ROS2 C++ driver should preserve the same separation of concerns:
  - Transport/protocol adapter
  - State/telemetry model
  - Command API
  - Mission-level orchestration layer

### 2.3 ROS2 / MoveIt2 / SMACC2 Considerations

- ROS2 examples emphasize composable nodes, topics/services/actions, and deterministic execution models.
- MoveIt2 contributes mature planning/execution patterns and action-based workflows; for a free-flying UAS in open air, MoveIt2 is optional for core flight control but valuable for future multi-agent, obstacle-aware, or manipulator-carrying UAS scenarios.
- SMACC2 is a strong fit for this problem because FCND is naturally an event-driven finite state machine and SMACC2 provides compile-time checked transitions and reusable client behaviors.

### 2.4 Current Implementation Status (March 2026)

The current `udacidrone_driver_cpp` implementation has established the following stack decisions and constraints:

- MAVLink translator uses a guarded include strategy:
	- Prefer `mavlink/v2.0/mavlink.h` with `MAVLINK_DIALECT` set to `common`.
	- Fallback to `mavlink/v2.0/common/mavlink.h`, `mavconn/mavlink_dialect.hpp`, and other compatible include paths.
	- Apply `#undef MAVLINK_VERSION` before including `mavros_msgs/mavlink_convert.hpp` to avoid C/C++ macro collisions.
- MAVLink APIs are consumed as C-style global symbols/macros (for example `mavlink_msg_command_long_pack`, `MAV_CMD_*`, `MAVLINK_MSG_ID_*`), not as `mavlink::` members.
- `mavlink_msg_set_position_target_local_ned_pack` must include the `time_boot_ms` argument for the Jazzy header signature.
- Build dependencies are explicitly declared and validated in the driver package:
	- CMake: `mavlink`, `mavros_msgs`, `libmavconn`
	- package.xml: `mavlink`, `mavros_msgs`, `libmavconn`
- Linkage should rely on `ament_target_dependencies(... libmavconn)` and should not hardcode `target_link_libraries(... mavconn)` unless the raw linker name is known to exist.
- Driver behavior parity note:
	- Component ID is fixed to `191` (onboard computer).
	- `cmdPosition` applies simulator/PX4 down-axis sign handling via `is_px4` mode.

Build note: MAVLink generated-header warnings about packed members are expected under `-Wpedantic` and are non-fatal unless warnings are promoted to errors.

## 3. Problem Statement

Design and implement a modular ROS2 C++ backend application that can command and monitor a Unity3D UAS simulator by creating a ROS2-native equivalent of the UdaciDrone API, then build mission workflows on top of that driver using SMACC2.

The solution must:

1. Reproduce FCND baseline mission behavior (3 m takeoff, rectangular waypoint traversal, landing, disarm).
2. Support additional mission modes (rectangle, star, and configurable patrol/surveillance path) using reusable planning modules.
3. Expose clear ROS2 interfaces (topics/services/actions/parameters) that decouple mission logic from transport details.
4. Run in Docker-based developer environments for consistent team workflows.
5. Be structured for extension to future autonomy modules (e.g., obstacle constraints, multi-UAS coordination, perception-in-the-loop).

## 4. Scope Definition

### In Scope

- ROS2 C++ driver package for simulator communication and command/control
- Telemetry translation and publication to ROS2 messages
- Command ingestion from ROS2 interfaces to MAVLink/Udaci-style command semantics
- SMACC2 mission state machine package with FCND-equivalent flow
- Mission plugins/pattern generators: rectangle, star, patrol path
- Simulation-first validation in Unity3D
- Docker build/runtime workflow

### Out of Scope (for Phase 1)

- Real hardware flight certification
- BVLOS/regulated operations
- Weaponization or invasive surveillance functionality
- Full MoveIt2 kinematic planning as a hard dependency for basic FCND-equivalent mission execution

## 5. System-Level Requirements

### Functional Requirements

1. Arm/disarm, guided/manual mode switching, takeoff, position command, land.
2. Receive and publish telemetry:
	- armed/guided/status
	- global position
	- local position (NED)
	- local velocity
	- optional attitude/IMU/barometer
3. Execute a rectangular mission equivalent to FCND task.
4. Execute a star-shaped waypoint mission.
5. Execute a configurable patrol mission (time/loop-limited) over user-defined waypoints.
6. Fail safely on connection timeout and mission abort requests.

### Non-Functional Requirements

1. Deterministic and observable state transitions.
2. Clear API boundaries between driver and mission layers.
3. Docker reproducibility across developer machines.
4. Testability at unit, integration, and simulation levels.
5. Parameter-driven behavior without recompilation.

## 6. Proposed Architecture

## 6.1 Package Breakdown

Create a ROS2 workspace with these packages:

1. `udacidrone_msgs`
	- Custom ROS2 interfaces for command/status events if existing standard msgs are insufficient.

2. `udacidrone_driver_cpp`
	- MAVLink/transport adapter and simulator connection manager.
	- Telemetry parser -> ROS2 publishers.
	- ROS2 services/actions/subscribers -> outbound flight commands.

3. `uas_mission_core`
	- Waypoint generation and mission definitions (rectangle/star/patrol).
	- Validation and constraint helpers.

4. `sm_uas_missions`
	- SMACC2 state machine package implementing mission workflows.
	- Orthogonals/clients for driver command execution and telemetry gating.

5. `uas_bringup`
	- Launch files, parameter YAMLs, runtime profiles, mission selection.

6. `uas_sim_docker`
	- Dockerfiles, compose manifests, environment bootstrapping scripts.

## 6.2 Logical Layering

- Layer A: Simulator Transport and Protocol
  - Handles TCP/UDP/WebSocket (as needed) and MAVLink framing.
- Layer B: ROS2 Driver API
  - Stable ROS2 interface for commands and telemetry.
- Layer C: Mission Planner
  - Generates ordered waypoints for mission shapes.
- Layer D: Mission Orchestrator (SMACC2)
  - Controls state transitions and safety checks.
- Layer E: Monitoring and Logging
  - ROS2 logs, bagging, mission metrics, transition traces.

## 7. ROS2 Interface Contract (Initial Draft)

### Topics (publish)

- `/uas/state` (armed, guided, status, connection)
- `/uas/local_position` (NED)
- `/uas/local_velocity` (NED)
- `/uas/global_position`
- `/uas/attitude` (optional)

### Topics (subscribe)

- `/uas/cmd_position`
- `/uas/cmd_velocity` (optional)
- `/uas/mission/select` (optional command channel)

### Services

- `/uas/arm`
- `/uas/disarm`
- `/uas/take_control`
- `/uas/release_control`
- `/uas/takeoff`
- `/uas/land`
- `/uas/abort_mission`

### Actions

- `/uas/fly_waypoints` (recommended for mission execution semantics)

### Parameters

- `connection_uri` (example: `tcp:127.0.0.1:5760`)
- `is_px4`
- `target_altitude_m`
- `arrival_position_tolerance_m`
- `arrival_velocity_tolerance_mps`
- `mission_mode` (`rectangle`, `star`, `patrol`)
- mission-specific geometry and timing parameters

## 8. Mission Modes

## 8.1 Rectangle Mission (FCND Baseline)

- Start at origin in local frame.
- Takeoff to 3 m.
- Traverse four corners of a rectangle (default 10 m x 10 m).
- Land and disarm.

## 8.2 Star Mission

- Generate star waypoints centered at current local origin.
- Parameterized by radius and number of points.
- Enforce minimum turn distance and waypoint spacing.

## 8.3 Patrol Mission (Simulation Safety-Oriented)

- Follow a predefined waypoint loop representing a patrol/observation route.
- Must include constraints:
  - geofence or soft-boundary checks
  - max mission duration
  - return/land on low confidence or timeout

Note: Implement this mode for simulation research and systems engineering evaluation, with explicit safety/privacy policy controls.

## 9. SMACC2 State Machine Design

Recommended top-level states:

1. `StManual`
2. `StArming`
3. `StTakeoff`
4. `StMissionNavigate`
5. `StLanding`
6. `StDisarming`
7. `StCompleted`
8. `StAbort`

Events (examples):

- `EvArmed`, `EvTakeoffReached`, `EvWaypointReached`, `EvMissionDone`, `EvLanded`, `EvDisarmed`
- `EvConnectionLost`, `EvTimeout`, `EvAbortRequested`

Orthogonals (examples):

- Flight command orthogonal (driver service/action client)
- Telemetry monitor orthogonal
- Safety watchdog orthogonal

This maps directly to FCND behavior while giving stronger modularity and compile-time transition validation.

## 10. Step-by-Step Development Plan

## Phase 0: Workspace and Tooling

1. Create ROS2 workspace and package skeletons.
2. Add Dockerfiles for dev and runtime.
3. Set up CI (format, build, tests).

## Phase 1: Driver Foundation (`udacidrone_driver_cpp`)

1. Implement simulator connection manager.
2. Implement telemetry decode and ROS2 publishers.
3. Implement core command services (arm/disarm/mode/takeoff/land/cmd_position).
4. Add connection timeout detection and fail-safe transitions.

Deliverable: Manual ROS2 command-line control of Unity drone.

## Phase 2: Mission Core (`uas_mission_core`)

1. Implement waypoint generators:
	- rectangle
	- star
	- patrol from parameterized waypoint list
2. Add waypoint acceptance criteria utilities (position/velocity thresholds).
3. Add mission schema validation.

Deliverable: Deterministic waypoint sets for each mission mode.

## Phase 3: SMACC2 Orchestration (`sm_uas_missions`)

1. Implement FCND-equivalent state flow.
2. Integrate mission mode selection.
3. Implement abort and timeout transitions.
4. Emit structured mission status and transition logs.

Deliverable: Event-driven autonomous missions with robust transition handling.

## Phase 4: Bringup and Simulation Validation (`uas_bringup`)

1. Build launch files for driver + state machine.
2. Add runtime parameter profiles for each mission mode.
3. Execute scenario tests in Unity3D simulator.
4. Capture rosbag and mission reports.

Deliverable: One-command mission launch for rectangle/star/patrol modes.

## Phase 5: Hardening

1. Add integration tests and mission regression suite.
2. Add fault-injection cases (connection loss, delayed telemetry).
3. Benchmark loop rates and transition latency.
4. Document operations and troubleshooting.

Deliverable: Production-like simulation stack with repeatable validation.

## 11. Testing and Validation Criteria

Minimum acceptance criteria:

1. Rectangle mission completes end-to-end with no manual intervention.
2. Star mission reaches all waypoints in order within tolerance.
3. Patrol mission runs bounded loops/time and lands safely on completion.
4. Connection loss triggers controlled abort/landing behavior.
5. All state transitions are logged and replayable from rosbag + logs.

Recommended test levels:

- Unit tests: waypoint generation, threshold logic, event guards.
- Integration tests: driver command-response behavior.
- System tests: full mission execution in Unity simulation.

## 12. Key Risks and Mitigations

1. MAVLink interpretation mismatch between simulator and PX4.
	- Mitigation: explicit adapter layer and simulator/PX4 mode flags.

2. Frame/sign convention errors (NED vs ENU, altitude/down sign).
	- Mitigation: centralized frame conversion utilities and tests.

3. State machine race conditions from asynchronous callbacks.
	- Mitigation: event queue discipline, watchdog timers, deterministic guards.

4. Tight coupling between mission logic and transport layer.
	- Mitigation: strict ROS2 interface contract and package boundaries.

5. Link-time mismatch on libmavconn symbol naming (`-lmavconn` vs exported package interface).
	- Mitigation: avoid hardcoded `target_link_libraries(... mavconn)` and rely on `ament_target_dependencies(... libmavconn)` exported linkage.

## 13. Deliverables

1. ROS2 C++ UdaciDrone-compatible driver package.
2. Mission-core package with rectangle/star/patrol generators.
3. SMACC2 mission state machine package with FCND baseline parity.
4. Bringup package with launch + parameter profiles.
5. Docker environment for reproducible development.
6. Test suite and mission validation report templates.

## 14. Definition of Done

The effort is complete when:

1. A developer can run Docker, launch Unity simulator + ROS2 stack, and execute all three mission modes.
2. FCND baseline behavior is demonstrably reproduced in ROS2 C++.
3. Mission logic is modular, parameterized, and independent of low-level transport internals.
4. Logs and tests provide objective evidence of mission correctness and failure handling.

## 15. Immediate Next Implementation Actions

1. Scaffold the ROS2 workspace and the six packages listed in Section 6.1.
2. Preserve the current MAVLink translator integration constraints from Section 2.4 while extending features.
3. Implement and test only Phase 1 command/telemetry loop first.
4. Add rectangle mission in SMACC2 to match FCND before adding star/patrol.
5. Freeze interface contracts, then scale to additional mission modes.

