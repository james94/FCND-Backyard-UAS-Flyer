# FCND Backyard Flyer — MVP Software Solution (Python)

## 1) Problem recap (what we must build)

Build an **event-driven state machine** (a `udacidrone.Drone` subclass) that autonomously flies a **10m box** at **3m altitude** in the Unity simulator.

The starter file already defines:

- The 6 flight states: `MANUAL → ARMING → TAKEOFF → WAYPOINT → LANDING → DISARMING → MANUAL`.
- Callback hooks for updates from the simulator: `LOCAL_POSITION`, `LOCAL_VELOCITY`, and `STATE`.
- Transition method stubs to send commands to the drone.

The MVP is complete when `python backyard_flyer.py` connects to the simulator, takes off, flies the box, lands, disarms, and ends the mission cleanly.

## 2) Key constraints and conventions

### 2.1 Reference frame (critical)

The simulator’s `local_position` is in **NED**:

- $x$ = **North** (meters)
- $y$ = **East** (meters)
- $z$ = **Down** (meters)

So “3 meters altitude” (3m above takeoff point) corresponds to:

- `down = -3.0`

Important nuance for this project:

- **Telemetry** is NED. At 3m altitude, you should observe `self.local_position[2] ≈ -3.0`.
- **Commands**: UdaciDrone `cmd_position(north, east, altitude, heading)` expects **altitude-up (positive)**.
	- Command `altitude = +3.0` for “3m up”.
	- Do **not** pass `down = -3.0` into `cmd_position()`; that mismatch commonly causes the vehicle to descend during waypoint flight.

### 2.2 Event-driven control loop

You do **not** run a while-loop that continuously checks state. Instead:

- The simulator streams messages.
- The `Drone` base class updates attributes.
- Your registered callbacks execute and decide whether to transition.

This makes the system reactive and simpler to reason about: “when new position arrives, check if we reached the target.”

## 3) MVP architecture

### 3.1 State machine responsibilities

Each state has two responsibilities:

1) **Command**: send the next action once when entering the state.
2) **Guard/transition**: wait for conditions to become true (via callbacks), then transition.

In the starter, “command on entry” is implemented via transition methods:

- `arming_transition()`
- `takeoff_transition()`
- `waypoint_transition()`
- `landing_transition()`
- `disarming_transition()`
- `manual_transition()` (already implemented)

### 3.2 Data model (minimum)

- `self.flight_state`: current `States` enum value
- `self.target_position`: current commanded target for `cmd_position()` as `[north, east, altitude]`
- `self.all_waypoints`: queue/list of remaining waypoints (each waypoint is `[north, east, altitude]`)
- `self.in_mission`: stops the mission when `False`

Optionally add small constants:

- `TARGET_ALTITUDE = 3.0`
- `BOX_SIZE = 10.0`
- `POS_TOL = 0.5` meters (arrival tolerance)
- `VEL_TOL = 0.5` m/s (nearly-stopped tolerance)

## 4) Step-by-step MVP implementation plan (mapped to `backyard_flyer.py`)

### Step 0 — Confirm callbacks are registered (already done)

In `__init__`, the starter correctly registers:

- `MsgID.LOCAL_POSITION → local_position_callback`
- `MsgID.LOCAL_VELOCITY → velocity_callback`
- `MsgID.STATE → state_callback`

These three callbacks are enough for an MVP.

### Step 1 — Implement `calculate_box()` to generate the 10m square

Goal: return waypoints for a square path **relative to the current local position**, at constant `altitude = 3.0` (altitude-up).

MVP approach:

1) Capture the current local position as the start reference:
	 - `north0 = self.local_position[0]`
	 - `east0 = self.local_position[1]`

2) Build a closed square (end where you started):

- Corner 1: `(north0 + 10, east0 + 0, 3)`
- Corner 2: `(north0 + 10, east0 + 10, 3)`
- Corner 3: `(north0 + 0,  east0 + 10, 3)`
- Corner 4: `(north0 + 0,  east0 + 0,  3)`

3) Return as a Python list of `np.array([n, e, altitude])`.

Design note: Keeping the square “anchored” to the takeoff point makes it robust even if you start from a different place in the map.

### Step 2 — Implement the transition methods (commands + state updates)

Each transition should (a) send a command to the drone and (b) update `self.flight_state`.

#### 2.1 `arming_transition()`

Entry action:

1) `self.take_control()` (guided mode)
2) `self.arm()`
3) Set home position to current global position **if your UdaciDrone version exposes a helper** (common patterns include `set_home_position(...)`).
	- If you don’t have a home-setting method available, you can omit this for the MVP; the local frame provided by the simulator will still work.
4) `self.flight_state = States.ARMING`

Why set home? It establishes a consistent reference for local coordinates and for logging/analysis.

#### 2.2 `takeoff_transition()`

Entry action:

1) Set `self.target_position = [current_north, current_east, 3.0]`.
2) `self.takeoff(3.0)` (API uses “altitude” in meters; the Drone handles the underlying frame).
3) `self.flight_state = States.TAKEOFF`

#### 2.3 `waypoint_transition()`

Entry action:

1) Pop the next waypoint from `self.all_waypoints`.
2) Set `self.target_position = waypoint`.
3) Command it:
	 - `self.cmd_position(north, east, altitude, heading)`
	 - Use `heading = 0.0` for MVP unless you want to yaw along the path.
4) `self.flight_state = States.WAYPOINT`

#### 2.4 `landing_transition()`

Entry action:

1) `self.land()`
2) `self.flight_state = States.LANDING`

#### 2.5 `disarming_transition()`

Entry action:

1) `self.disarm()`
2) `self.flight_state = States.DISARMING`

`manual_transition()` is already implemented and ends the mission cleanly.

### Step 3 — Implement callback logic (guards / transitions)

Callbacks should be “if in state X, check condition Y, then call transition Z”.

#### 3.1 `state_callback()` (armed/guided changes)

Use it for state changes that depend on arming/disarming:

- If `flight_state == States.MANUAL` and `in_mission` is true: start the mission by calling `arming_transition()`.
- If `flight_state == States.ARMING` and `self.armed` is true: call `takeoff_transition()`.
- If `flight_state == States.DISARMING` and `not self.armed`: call `manual_transition()`.

Design note: You can also gate on `self.guided` in ARMING, but for MVP “armed is true” is often sufficient.

#### 3.2 `local_position_callback()` (position updates)

Use it for “arrived at altitude” and “arrived at waypoint” checks.

Takeoff completion:

- If `flight_state == States.TAKEOFF`:
	- Compute current altitude: `alt = -self.local_position[2]`
	- If `alt > 0.95 * 3.0`:
		- Set `self.all_waypoints = self.calculate_box()`
		- Call `waypoint_transition()`

Waypoint arrival:

- If `flight_state == States.WAYPOINT`:
	- Compare horizontal distance to target:
		- `d_ne = np.linalg.norm(self.local_position[0:2] - self.target_position[0:2])`
	- If `d_ne < POS_TOL`:
		- If waypoints remain: call `waypoint_transition()`
		- Else: call `landing_transition()`

Why horizontal distance only? At MVP level you’re already commanding a constant altitude; checking XY is usually stable.

#### 3.3 `velocity_callback()` (velocity updates)

Use it for landing completion (on ground and nearly stopped):

- If `flight_state == States.LANDING`:
	- If `abs(self.local_position[2]) < 0.1` (near ground) AND
		`np.linalg.norm(self.local_velocity[0:2]) < VEL_TOL`:
		- Call `disarming_transition()`

This helps avoid disarming while still descending or sliding.

## 5) Expected execution flow (end-to-end)

1) Script starts; connection begins streaming messages.
2) First `STATE` callback fires → `arming_transition()`.
3) Drone becomes armed → `state_callback()` triggers `takeoff_transition()`.
4) Drone climbs; `LOCAL_POSITION` updates until altitude reached → compute waypoints → `waypoint_transition()`.
5) Each time the drone reaches a corner → command next waypoint.
6) After final corner → `landing_transition()`.
7) During landing, `LOCAL_VELOCITY` / `LOCAL_POSITION` show near-ground and near-stationary → `disarming_transition()`.
8) `STATE` indicates disarmed → `manual_transition()` releases control, stops logs, ends mission.

## 6) MVP verification checklist (what to observe)

- Takeoff reaches approximately 3m altitude.
- Drone flies a square with ~10m edges.
- Drone lands close to the start point.
- `NavLog.txt` is created in `Logs/` (from `start_log`).
- Script exits without needing to kill the process.

## 7) Common pitfalls (and the MVP fixes)

- **Telemetry-vs-command mismatch**: telemetry is NED (`altitude = -down`), but `cmd_position()` expects altitude-up.
	- Observe ~3m up as `self.local_position[2] ≈ -3.0`, but command `self.cmd_position(..., altitude=3.0, ...)`.
- **No tolerance**: exact equality on floating sensors never triggers; use `POS_TOL` and thresholds like `0.95 * altitude`.
- **Command spam**: don’t re-send commands every callback tick; only send on state transitions.
- **Landing disarm too early**: require near-ground + low velocity before disarming.

## 8) Minimal code footprint (what you actually implement)

To satisfy the project requirements, you only need to fill in:

- `local_position_callback()`
- `velocity_callback()`
- `state_callback()`
- `calculate_box()`
- `arming_transition()`
- `takeoff_transition()`
- `waypoint_transition()`
- `landing_transition()`
- `disarming_transition()`

Everything else in the starter can remain unchanged for an MVP.

