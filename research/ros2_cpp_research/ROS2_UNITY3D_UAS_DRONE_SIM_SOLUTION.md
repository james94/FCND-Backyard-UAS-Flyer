# ROS2 Unity3D UAS Drone Simulator Solution Guide

## 1) Purpose

This document is the implementation playbook for the approved software problem statement. It explains how to build a modular ROS2 Jazzy C++ backend that interacts with the Unity3D UAS simulator, reproduces FCND Backyard Flyer behavior, and extends to additional mission modes using SMACC2.

The goal is to move from concept to a reproducible engineering workflow with clear checkpoints, interfaces, and test criteria.

## 2) Solution Overview

The solution uses five functional layers:

1. Simulator transport and MAVLink adapter
2. ROS2 driver interface for telemetry and commands
3. Mission planning core for waypoint generation
4. SMACC2 state machine orchestration
5. Bringup, logging, testing, and containerization

The architecture intentionally separates protocol details from mission logic so new missions can be added without rewriting low-level communication.

## 3) Workspace and Package Layout

Create or extend a ROS2 workspace with the following packages:

1. udacidrone_msgs
2. udacidrone_driver_cpp
3. uas_mission_core
4. sm_uas_missions
5. uas_bringup
6. uas_sim_docker

Recommended top-level structure:

- src/
  - udacidrone_msgs/
  - udacidrone_driver_cpp/
  - uas_mission_core/
  - sm_uas_missions/
  - uas_bringup/
  - uas_sim_docker/

## 4) Step-by-Step Implementation Plan

## Phase 0: Environment and Tooling Setup

Objective:
Establish reproducible development and build foundations.

Steps:

1. Install ROS2 Jazzy on Ubuntu 24.04 host and verify environment sourcing.
2. Install build tools: colcon, rosdep, vcstool, CMake, compiler toolchain.
3. Create workspace and initialize package skeletons.
4. Set formatting and linting policy for C++ and ROS2 package metadata.
5. Add a Docker development image based on Ubuntu 24.04 with ROS2 Jazzy preinstalled.

Exit criteria:

1. Workspace builds with empty package stubs.
2. Docker image builds successfully and can run colcon build.

## Phase 1: Build the ROS2 C++ Driver

Objective:
Create a ROS2-native driver that mirrors UdaciDrone control and telemetry semantics.

### 1.1 Driver Node Responsibilities

Implement a main driver node that:

1. Opens simulator connection using configured URI.
2. Receives MAVLink/Udaci style telemetry.
3. Publishes ROS2 telemetry topics.
4. Exposes ROS2 services for arm, disarm, mode switching, takeoff, and land.
5. Subscribes to local position command topic.
6. Handles connection timeout and safe error state.

### 1.2 Initial ROS2 Interface Contract

Publish topics:

1. /uas/state
2. /uas/local_position
3. /uas/local_velocity
4. /uas/global_position
5. /uas/attitude (optional in first pass)

Subscribe topics:

1. /uas/cmd_position
2. /uas/cmd_velocity (optional)

Services:

1. /uas/arm
2. /uas/disarm
3. /uas/take_control
4. /uas/release_control
5. /uas/takeoff
6. /uas/land
7. /uas/abort_mission

### 1.3 Implementation Notes

1. Keep a single source of truth for state fields (armed, guided, status, positions, velocity).
2. Add explicit handling for frame/sign conventions (NED and altitude/down differences).
3. Implement a connection watchdog timer.
4. On timeout, publish degraded state and raise mission abort event channel.

Exit criteria:

1. Operator can arm, takeoff, command local position, and land from ROS2 interfaces.
2. Telemetry topics update continuously while simulator is running.

## Phase 2: Mission Planning Core

Objective:
Provide reusable waypoint generation independent of the state machine.

### 2.1 Mission Planner Module

Add mission planner classes in uas_mission_core:

1. Rectangle planner
2. Star planner
3. Patrol planner (parameterized waypoint loop)

### 2.2 Planner Inputs and Outputs

Inputs:

1. Start pose or local origin
2. Target altitude
3. Shape-specific geometry parameters
4. Limits such as max leg length and waypoint count

Outputs:

1. Ordered list of 3D waypoints in local frame
2. Optional heading values per waypoint

### 2.3 Validation Rules

1. Reject empty waypoint plans.
2. Reject waypoints outside configured geofence.
3. Validate altitude bounds.
4. Validate max mission duration estimate.

Exit criteria:

1. All three mission planners produce deterministic outputs from fixed inputs.
2. Unit tests cover geometry and boundary validation.

## Phase 3: SMACC2 Mission Orchestration

Objective:
Implement event-driven mission execution aligned with FCND behavior.

### 3.1 State Machine Topology

Recommended states:

1. StManual
2. StArming
3. StTakeoff
4. StMissionNavigate
5. StLanding
6. StDisarming
7. StCompleted
8. StAbort

### 3.2 Core Events

1. EvArmed
2. EvTakeoffReached
3. EvWaypointReached
4. EvMissionDone
5. EvLanded
6. EvDisarmed
7. EvConnectionLost
8. EvTimeout
9. EvAbortRequested

### 3.3 Orthogonals and Client Behaviors

Define orthogonals for:

1. Flight command client to call driver services/actions
2. Telemetry monitor to evaluate transition guards
3. Safety watchdog to trigger abort transitions

Create reusable behaviors:

1. CbArm
2. CbTakeoff
3. CbFlyWaypoint
4. CbLand
5. CbDisarm
6. CbAbort

### 3.4 Transition Guard Examples

1. Takeoff complete when local altitude reaches target within tolerance.
2. Waypoint reached when position error and velocity error are both under thresholds.
3. Landing complete when near ground and vertical velocity is small.

Exit criteria:

1. Rectangle mission completes autonomously with no manual intervention.
2. Abort path is deterministic and always reaches safe terminal state.

## Phase 4: Bringup and Runtime Profiles

Objective:
Enable one-command launch for each mission mode.

Steps:

1. Create launch files in uas_bringup for driver plus SMACC2 machine.
2. Add YAML profiles for rectangle, star, and patrol mission settings.
3. Add runtime options for simulator URI, mission mode, tolerances, and geofence.
4. Add startup checks that fail fast when required parameters are missing.

Exit criteria:

1. A user can select mission mode by parameter and launch end-to-end stack.
2. Mission status and transitions are visible in logs.

## Phase 5: Docker and Developer Workflow

Objective:
Ensure reproducible local and team development.

Steps:

1. Define dev container image with ROS2 Jazzy and build dependencies.
2. Define runtime image for mission execution.
3. Add compose file for backend stack and optional auxiliary tools.
4. Mount workspace for iterative development.
5. Add scripts for build, test, and launch.

Recommended script set:

1. build.sh
2. test.sh
3. run_rectangle.sh
4. run_star.sh
5. run_patrol.sh

Exit criteria:

1. Fresh machine can clone, build container, and run mission with minimal manual setup.

## 5) Detailed Mission Mode Implementation

## Rectangle mode

Behavior:

1. Arm and switch to guided/offboard equivalent.
2. Takeoff to 3 m.
3. Visit rectangle corners.
4. Land and disarm.

Default geometry:

1. Width 10 m
2. Length 10 m

## Star mode

Behavior:

1. Generate star vertices around local origin.
2. Visit ordered vertices.
3. Land and disarm.

Suggested parameters:

1. Outer radius
2. Inner radius ratio
3. Number of star points

## Patrol mode

Behavior:

1. Load waypoint list from YAML.
2. Loop route for configured cycles or duration.
3. Trigger safe return and land on timeout or abort.

Required safety controls:

1. Max mission time
2. Geofence bounds
3. Connection watchdog
4. Operator abort service

## 6) Testing Strategy

## Unit tests

1. Waypoint geometry for rectangle, star, patrol.
2. Threshold and guard computations.
3. Frame conversion utilities.

## Integration tests

1. Driver topic and service behavior.
2. Command-to-telemetry loop closure.
3. Connection loss handling.

## System tests in Unity3D

1. Rectangle baseline mission.
2. Star mission.
3. Patrol mission.
4. Fault injection: delayed telemetry and dropped connection.

## 7) Operational Observability

Track and store:

1. Mission start and end times
2. State transition timeline
3. Waypoint completion timestamps
4. Abort reasons
5. Connection health events

Recommended outputs:

1. ROS2 logs
2. Rosbag recordings
3. Mission summary report artifact per run

## 8) Risk Controls and Engineering Decisions

1. Protocol differences between simulator and PX4 can cause command interpretation mismatches.
	- Decision: isolate transport behavior in the driver adapter and gate by configuration.

2. NED vs ENU and altitude/down sign errors are common integration failures.
	- Decision: centralize frame conversion logic and enforce unit tests for every conversion path.

3. Asynchronous callback timing can create race conditions.
	- Decision: route transition triggers through explicit SMACC2 events and guard conditions.

4. Mission logic can become tightly coupled to communication details.
	- Decision: mission planners produce generic waypoint goals and never call transport code directly.

## 9) Recommended Build Order (Execution Checklist)

1. Stand up driver skeleton with connection and basic telemetry publishing.
2. Add arm, takeoff, land, and cmd_position command path.
3. Validate manual CLI-driven mission sequence.
4. Build rectangle planner and execute using a minimal mission executor.
5. Integrate SMACC2 state machine for FCND parity flow.
6. Add star and patrol planners.
7. Harden abort and timeout handling.
8. Add launch profiles and Docker scripts.
9. Run full regression scenarios and document outcomes.

## 10) Definition of Ready for Coding

Coding should begin when:

1. Interface contracts are reviewed and frozen for Phase 1.
2. Team agrees on frame conventions and tolerances.
3. Mission mode parameter schema is agreed.
4. Docker base image and CI baseline are available.

## 11) Definition of Done for Solution Delivery

The solution is complete when:

1. Rectangle, star, and patrol missions run in Unity3D from ROS2 launch.
2. FCND baseline behavior is reproduced in ROS2 C++.
3. Abort and connection-loss safety paths are verified.
4. Tests and logs provide objective evidence of correctness.
5. New mission modes can be added without editing driver internals.

## 12) Immediate Next Actions

1. Implement Phase 1 driver node and command services first.
2. Execute an end-to-end rectangle mission before adding star and patrol.
3. Integrate SMACC2 once driver and planner contracts are stable.
4. Add Docker and CI after first successful mission pass, then harden.

