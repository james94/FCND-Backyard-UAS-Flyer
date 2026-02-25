# FCND Backyard Flyer — Solution Design (Software Engineering)

## 1) What is the problem (requirements distilled from README)

You need to implement an **autonomous flight controller** (Python) that:

1) Connects to the Unity quadcopter simulator via MAVLink (through the UdaciDrone API).
2) Runs an **event-driven state machine**.
3) Commands a **10 meter box (square)** at **3 meter altitude**, then lands and ends the mission.

The project scaffolding is provided in `backyard_flyer.py`:

- A `BackyardFlyer(Drone)` subclass.
- An enum of flight states: `MANUAL, ARMING, TAKEOFF, WAYPOINT, LANDING, DISARMING`.
- Three callback stubs triggered by simulator messages:
  - `local_position_callback()` for `MsgID.LOCAL_POSITION`
  - `velocity_callback()` for `MsgID.LOCAL_VELOCITY`
  - `state_callback()` for `MsgID.STATE`
- Transition method stubs where you send commands and change state.

Success is not “the code runs”; success is that the drone **physically flies the box** in sim and terminates cleanly.

---

## 2) Key technical constraints (the stuff that breaks if you ignore it)

### 2.1 Local coordinates are NED (North-East-Down)

`self.local_position` is in meters in **NED** coordinates:

- `north = local_position[0]`
- `east  = local_position[1]`
- `down  = local_position[2]`

Altitude is “up”, but local `down` is positive downward.

So a target altitude of 3 meters above ground is:

- `target_down = -3.0`

Important nuance for this project:

- **Telemetry** (`self.local_position`) is NED, so 3m altitude corresponds to `down = -3.0`.
- **Commands**: UdaciDrone `cmd_position(north, east, altitude, heading)` expects **altitude-up (positive)**.
	- A common bug is sending `down=-3.0` to `cmd_position()`, which causes the vehicle to descend during waypoint flight.

### 2.2 The controller is event-driven

You do not poll in a tight loop. Your “control loop” is:

1) Simulator sends messages.
2) UdaciDrone updates attributes.
3) Your callbacks fire and run transition logic.

This is a classic reactive system: state + new measurement → decide next command.

### 2.3 A flight controller is safety-critical by default

Even in a simulator, design for:

- idempotent transitions (don’t spam commands every callback tick)
- tolerances (never compare floats for equality)
- clear termination (always stop logs + connection)

---

## 3) Proposed software architecture

### 3.1 Component responsibilities

**Drone API (provided):**

- Communication (MAVLink)
- Attribute updates (position, velocity, armed/guided)
- Command methods (arm, takeoff, cmd_position, land, disarm, stop)

**Your layer (this project):**

- State machine (when to send which commands)
- Waypoint planning for the box
- Transition guards and tolerances
- Mission lifecycle (start → execute → stop)

### 3.2 Control flow overview

The state machine is split into two kinds of functions:

1) **Transition functions** (send commands once, update `flight_state`).
2) **Callbacks** (when new data arrives, check if a transition guard is met).

This separation is a software engineering best practice:

- Transition functions are “actions”
- Callbacks are “guards”

It keeps code deterministic and testable.

---

## 4) Step-by-step design → development approach

### Step 1 — Define state machine transitions (what commands happen on entry)

Each transition method should:

- print/log a clear message
- send **one** command to the drone
- set `self.flight_state` (matches the starter code’s variable name) to the new state

#### 1.1 MANUAL → ARMING (`arming_transition`)

Entry behavior:

- `take_control()` (guided)
- `arm()`
- (Optional, API-dependent) set home position to current global position
- `flight_state = ARMING`

Guard to leave ARMING:

- `self.armed == True` (and optionally `self.guided == True`)

#### 1.2 ARMING → TAKEOFF (`takeoff_transition`)

Entry behavior:

- set `target_position` to current north/east and `altitude = 3.0`
- command `takeoff(3.0)`
- `flight_state = TAKEOFF`

Guard to leave TAKEOFF:

- altitude reached (e.g., `-local_position[2] >= 0.95 * 3.0`)

#### 1.3 TAKEOFF → WAYPOINT (`waypoint_transition`)

Entry behavior:

- generate the box waypoints once (`calculate_box`) if not already, storing them in `self.all_waypoints`
- pop the next waypoint from `self.all_waypoints`
- send `cmd_position(north, east, altitude, heading)`
- `flight_state = WAYPOINT`

Guard to leave WAYPOINT:

- horizontal distance to target within tolerance (e.g., < 0.5m)

#### 1.4 WAYPOINT → LANDING (`landing_transition`)

Entry behavior:

- send `land()`
- `flight_state = LANDING`

Guard to leave LANDING:

- near ground and nearly stopped (position + velocity checks)

#### 1.5 LANDING → DISARMING (`disarming_transition`)

Entry behavior:

- send `disarm()`
- `flight_state = DISARMING`

Guard to leave DISARMING:

- `self.armed == False` then execute `manual_transition()`

### Step 2 — Design the waypoint generator (`calculate_box`)

Requirement: a **10m box** at **3m altitude**.

Minimal design:

- Use the current local position as the origin reference `(n0, e0)`.
- Waypoints should use **altitude-up (positive)** for `cmd_position` (e.g., `altitude = 3.0`).
	- You still use NED `down=-3.0` only when interpreting `local_position`.
- Generate 4 corners (and optionally return to start):

1) `(n0 + 10, e0 + 0,  -3)`
2) `(n0 + 10, e0 + 10, -3)`
3) `(n0 + 0,  e0 + 10, -3)`
4) `(n0 + 0,  e0 + 0,  -3)`

Represent each waypoint as an `np.array([north, east, down])`.

Engineering note:

- Using **relative** waypoints avoids depending on a specific map location.
- Keeping `down` constant simplifies guard logic.

### Step 3 — Implement guard logic in callbacks (reactive transitions)

Callbacks should be short and state-specific; pattern:

```text
if flight_state == X and condition_met:
	 transition_to_next_state()
```

#### 3.1 `state_callback()`

Best use: changes dependent on `armed/guided`.

Recommended MVP logic:

- If `flight_state == MANUAL` and `in_mission` is True → call `arming_transition()`.
- If `flight_state == ARMING` and `armed` is True → call `takeoff_transition()`.
- If `flight_state == DISARMING` and `armed` is False → call `manual_transition()`.

#### 3.2 `local_position_callback()`

Best use: altitude and waypoint arrival.

- If `flight_state == TAKEOFF` and altitude reached → compute waypoints and call `waypoint_transition()`.
- If `flight_state == WAYPOINT` and arrived at target:
  - if more waypoints → `waypoint_transition()`
  - else → `landing_transition()`

Guard computation recommendations:

- Use horizontal distance: `||pos_NE - target_NE||`.
- Use a position tolerance constant like `POS_TOL = 0.5`.

#### 3.3 `velocity_callback()`

Best use: finishing landing.

- If `flight_state == LANDING` and near ground + low speed → `disarming_transition()`.

Use tolerances:

- `abs(local_position[2]) < 0.1` (close to ground)
- `norm(local_velocity[0:2]) < 0.5` (nearly stopped)

### Step 4 — Add engineering “guard rails” (MVP-level)

Even for MVP, add a few safety-oriented practices:

- **Idempotence:** transition functions should only be called when you truly want to change state.
- **Waypoint queue:** pop exactly one waypoint per waypoint transition.
- **Telemetry logging:** use the provided `start_log` / `stop_log` lifecycle.
- **Time-to-first-command:** allow a short startup delay (already in `__main__` with `time.sleep(2)`).

---

## 5) MVP definition (what “done” looks like)

### MVP scope

The MVP is intentionally small:

- Single mission profile (10m box at 3m altitude)
- No dynamic replanning
- No obstacle handling
- Constant yaw/heading
- Minimal tolerances

### MVP acceptance criteria

- Connects successfully to simulator
- Arms and takes off to ~3m
- Flies 4 legs that are ~10m each
- Lands and disarms
- Exits without manual kill

### MVP implementation list (exact TODOs to fill)

- `local_position_callback`
- `velocity_callback`
- `state_callback`
- `calculate_box`
- `arming_transition`
- `takeoff_transition`
- `waypoint_transition`
- `landing_transition`
- `disarming_transition`

---

## 6) Modularization approach (how to engineer this beyond a script)

Once the MVP works, the most valuable engineering step is to separate concerns.

### 6.1 Modules you’d introduce (still Python)

1) **Mission config** (`config.py`)
	- box size, altitude, tolerances, headings

2) **Waypoint planner** (`planner.py`)
	- `build_box(origin_ne, box_size, altitude_m) -> list[Waypoint]`
	  - Plan waypoints using **altitude-up** (matches `cmd_position`), and interpret telemetry altitude as `-local_position[2]`.

3) **State machine core** (`state_machine.py`)
	- pure functions for guards:
	  - `reached_altitude(local_position, target_alt)`
	  - `reached_waypoint(local_position, target_position, tol)`
	  - `ready_to_disarm(local_position, local_velocity)`

4) **Adapter layer** (`drone_adapter.py`)
	- wraps UdaciDrone methods; allows swapping sim vs real

### 6.2 Why modular matters in autonomous systems

- Makes it testable: planners/guards can be unit-tested without a simulator.
- Makes it portable: same planner and guards can run against a different vehicle API.
- Makes it safer: you can reason about deterministic parts separately from I/O.

---

## 7) MiNiFi C++/Python → NiFi evolution approach (edge-to-platform)

This section is an **optional engineering evolution path** (not required to pass the Udacity project).

This project is a real-time control loop. **NiFi should not be in the actuation loop**.

The right split is:

- **Real-time control:** stays on the drone computer (this Python controller / PX4).
- **Data movement + observability + analytics:** MiNiFi/NiFi.

### 7.1 MVP (single-process)

- Controller writes telemetry logs locally (already via `start_log`).
- Post-run, you manually inspect logs.

### 7.2 Modular software + structured telemetry (still local)

Upgrade the controller to emit structured events (e.g., JSON lines):

- state transitions
- commanded waypoints
- measured position/velocity
- mission outcome and timings

This can be written to a local file or a local message queue.

### 7.3 Introduce MiNiFi at the edge (near the drone)

**MiNiFi** is a lightweight dataflow agent designed for edge devices.

Edge pattern:

1) The controller produces telemetry artifacts:
	- files (log/JSON)
	- or a local TCP/UDP stream
2) MiNiFi tails/ingests these artifacts.
3) MiNiFi buffers and forwards to a central system.

Why MiNiFi here:

- Handles intermittent connectivity.
- Backpressure and buffering are built-in.
- You can deploy the same “flow concept” across devices.

Implementation choices:

- **MiNiFi C++** on constrained Linux (typical for edge)
- **MiNiFi Java** if you have more resources and want feature parity

### 7.4 Forward to central NiFi (platform ingestion)

**NiFi** becomes the orchestration layer for:

- collecting telemetry from many drones
- routing, enriching, and storing data
- triggering alerts and dashboards

Common flow concepts (technology-agnostic):

- ingest telemetry
- validate schema
- enrich with mission metadata
- store to:
  - object storage (raw logs)
  - time-series DB (metrics)
  - data lake / warehouse (analytics)

### 7.5 Where Python fits in a NiFi/MiNiFi pipeline

Python is ideal for:

- feature extraction from logs
- anomaly detection prototypes
- offline model training

But keep ML/analytics **out of the real-time flight safety path** unless you can prove deterministic behavior.

### 7.6 Evolution summary (practical roadmap)

1) **MVP:** event-driven state machine controlling the sim.
2) **Modular:** separate planning/guards/config + structured telemetry.
3) **Edge:** MiNiFi ships telemetry with buffering/backpressure.
4) **Platform:** NiFi routes + stores + alerts + powers analytics.

---

## 8) What I would test (engineering validation)

Even without heavy tooling, validate in layers:

1) **Unit tests (pure logic):**
	- waypoint generation is correct length and coordinates
	- arrival checks behave with tolerance
2) **Integration tests (sim):**
	- mission reaches each state in order
	- no state gets “stuck”
3) **Log review:**
	- state transition timestamps
	- position/velocity sanity
	- mission duration and corner arrival times

