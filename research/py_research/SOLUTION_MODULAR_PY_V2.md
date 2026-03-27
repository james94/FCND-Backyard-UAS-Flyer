# Backyard Flyer — Modular, Class-Based Python Solution (Follow-Along Guide)

This document turns the MVP state machine (in `backyard_flyer.py`) into a **modular, class-based** Python design that you can type out manually in a new file/package and learn from.

It is intentionally:

- Minimal (only what the README requires: 10m box @ 3m altitude)
- Testable (planner + guards are pure functions/classes)
- Portable (UdaciDrone specifics live behind a small adapter)

> Reminder on frames: `local_position` is **NED** (so at 3m altitude, `down = -3.0`).
> 
> **Important:** UdaciDrone’s `cmd_position(north, east, altitude, heading)` expects **altitude-up (positive)**.
> If you pass `down=-3.0` into `cmd_position()`, the vehicle will try to descend during waypoint flight.

---

## 0) Target end state (what you’ll have)

You will end up with:

1) A thin entry script (keeps Udacity workflow) that starts the mission.
2) A small package with:
   - `MissionConfig` constants
	- `SensorLayer` to normalize simulator telemetry into a consistent frame
	- `PerceptionLayer` for lightweight detection/localization estimates
	- `PlannerStack` with Route Planning, Prediction, Behavior Planning, and Trajectory Planning
	- `ControlLayer` that converts trajectory targets into low-level flight commands
   - `FlightGuards` to decide “have we arrived / can we disarm”
   - `BackyardFlyerFSM` (the state machine)
   - `UdacidroneAdapter` (a wrapper around `Drone` methods)
3) Unit tests for planner + guards (+ optional sensor/perception smoke tests).

You can implement this either:

- **Option A (recommended):** Keep `backyard_flyer.py` as the entrypoint and have it import your modular classes.
- **Option B:** Create a new `backyard_flyer_modular.py` for your own use.

---

## 1) Suggested folder layout (Stage 1 modular)

Use the structure from `SOLUTION_DESIGN_FOLDER_STRUCTURE.md`:

```text
FCND-Backyard-UAS-Flyer/
  backyard_flyer.py
  src/
	backyard_flyer/
	  __init__.py
	  config.py
	  types.py
	  sensing.py
	  perception.py
	  planner.py
	  control.py
	  guards.py
	  adapter.py
	  fsm.py
	  app.py
	  planning/
		__init__.py
		route_planner.py
		predictor.py
		behavior_planner.py
		trajectory_planner.py
  tests/
	test_planning.py
	test_guards.py
```

If you don’t want packaging right now, you can still follow along by putting these “modules” into separate files in a single folder.

---

## 2) Module-by-module implementation (copyable skeletons)

### 2.1 `config.py` — mission constants

Purpose: keep mission parameters in one place.

```python
from dataclasses import dataclass


@dataclass(frozen=True)
class MissionConfig:
	box_size_m: float = 10.0
	target_altitude_m: float = 3.0

	# tolerances
	pos_tolerance_m: float = 0.5
	vel_xy_tolerance_mps: float = 0.5
	ground_down_tolerance_m: float = 0.1

	# takeoff completion threshold
	takeoff_altitude_ratio: float = 0.95

	# heading to maintain during cmd_position
	heading_rad: float = 0.0
```

Design notes:

- `@dataclass(frozen=True)` makes config immutable (safer).
- Keep values simple for MVP; you can tune later.

---

### 2.2 `types.py` — small, explicit data types

Purpose: reduce “mystery arrays” and make intent clear.

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class NED:
	north: float
	east: float
	down: float

	def as_np(self) -> np.ndarray:
		return np.array([self.north, self.east, self.down], dtype=float)

	@staticmethod
	def from_iterable(values: Iterable[float]) -> "NED":
		v = list(values)
		return NED(float(v[0]), float(v[1]), float(v[2]))


@dataclass(frozen=True)
class Waypoint:
	"""Waypoint in local N/E with altitude-up (meters).

	This matches UdaciDrone `cmd_position()` which takes altitude (not NED down).
	"""

	north: float
	east: float
	altitude_m: float
```

You can keep using `np.array` everywhere if you prefer; the key is consistency.

---

### 2.3 `planner.py` — Planning layer orchestrator (Route + Prediction + Behavior + Trajectory)

Purpose: keep your Python autonomy backend explicitly layered.

- Route Planning: generates a geometric route (the 10m box in MVP).
- Prediction: estimates near-future vehicle state over a short horizon.
- Behavior Planning: decides intent from mission phase + predictions.
- Trajectory Planning: produces a time-ordered flyable trajectory.

This keeps the current project simple while matching a production-style autonomy stack.

```python
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List

import numpy as np

from .config import MissionConfig
from .types import Waypoint


@dataclass(frozen=True)
class RoutePlan:
	waypoints: List[Waypoint]


@dataclass(frozen=True)
class PredictedState:
	north: float
	east: float
	altitude_m: float
	vn: float
	ve: float
	vz: float


class BehaviorMode(Enum):
	HOLD = "hold"
	TRACK_ROUTE = "track_route"
	LAND_NOW = "land_now"


@dataclass(frozen=True)
class BehaviorCommand:
	mode: BehaviorMode
	target_waypoint: Waypoint | None


@dataclass(frozen=True)
class TrajectoryPoint:
	t_s: float
	north: float
	east: float
	altitude_m: float
	heading_rad: float


@dataclass(frozen=True)
class Trajectory:
	points: List[TrajectoryPoint]


@dataclass
class RoutePlanner:
	cfg: MissionConfig

	def build_box_route(self, origin_north: float, origin_east: float) -> RoutePlan:
		s = self.cfg.box_size_m
		wps = [
			Waypoint(origin_north + s, origin_east + 0.0, self.cfg.target_altitude_m),
			Waypoint(origin_north + s, origin_east + s, self.cfg.target_altitude_m),
			Waypoint(origin_north + 0.0, origin_east + s, self.cfg.target_altitude_m),
			Waypoint(origin_north + 0.0, origin_east + 0.0, self.cfg.target_altitude_m),
		]
		return RoutePlan(waypoints=wps)


@dataclass
class Predictor:
	def one_step(self, local_position: np.ndarray, local_velocity: np.ndarray, dt_s: float = 0.2) -> PredictedState:
		n = float(local_position[0] + local_velocity[0] * dt_s)
		e = float(local_position[1] + local_velocity[1] * dt_s)
		alt = float(-(local_position[2] + local_velocity[2] * dt_s))
		return PredictedState(
			north=n,
			east=e,
			altitude_m=alt,
			vn=float(local_velocity[0]),
			ve=float(local_velocity[1]),
			vz=float(local_velocity[2]),
		)


@dataclass
class BehaviorPlanner:
	def decide(self, mission_phase: str, predicted: PredictedState, next_waypoint: Waypoint | None) -> BehaviorCommand:
		if mission_phase == "landing":
			return BehaviorCommand(mode=BehaviorMode.LAND_NOW, target_waypoint=None)
		if next_waypoint is None:
			return BehaviorCommand(mode=BehaviorMode.HOLD, target_waypoint=None)
		return BehaviorCommand(mode=BehaviorMode.TRACK_ROUTE, target_waypoint=next_waypoint)


@dataclass
class TrajectoryPlanner:
	cfg: MissionConfig

	def build(self, behavior: BehaviorCommand, now_ned: np.ndarray) -> Trajectory:
		if behavior.mode != BehaviorMode.TRACK_ROUTE or behavior.target_waypoint is None:
			hold = TrajectoryPoint(
				t_s=0.0,
				north=float(now_ned[0]),
				east=float(now_ned[1]),
				altitude_m=float(-now_ned[2]),
				heading_rad=self.cfg.heading_rad,
			)
			return Trajectory(points=[hold])

		tgt = behavior.target_waypoint
		goal = TrajectoryPoint(
			t_s=1.0,
			north=tgt.north,
			east=tgt.east,
			altitude_m=tgt.altitude_m,
			heading_rad=self.cfg.heading_rad,
		)
		return Trajectory(points=[goal])


@dataclass
class PlannerStack:
	route: RoutePlanner
	predictor: Predictor
	behavior: BehaviorPlanner
	trajectory: TrajectoryPlanner

	def plan_cycle(
		self,
		mission_phase: str,
		local_position: np.ndarray,
		local_velocity: np.ndarray,
		next_waypoint: Waypoint | None,
	) -> Trajectory:
		pred = self.predictor.one_step(local_position, local_velocity)
		cmd = self.behavior.decide(mission_phase, pred, next_waypoint)
		return self.trajectory.build(cmd, now_ned=local_position)
```

Design notes:

- You still get the same 10m box behavior, but now planning responsibilities are explicit.
- This decomposition is the right place to scale into obstacle handling later.

---

### 2.4 `sensing.py` — Sensor layer (telemetry ingestion and normalization)

Purpose: ingest raw simulator data from the adapter and convert it into one consistent sensor snapshot for autonomy.

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class SensorFrame:
	time_s: float
	local_position_ned: np.ndarray
	local_velocity_ned: np.ndarray
	global_position: np.ndarray
	armed: bool
	guided: bool


@dataclass
class SensorLayer:
	last_frame: Optional[SensorFrame] = None

	def ingest(
		self,
		time_s: float,
		local_position: np.ndarray,
		local_velocity: np.ndarray,
		global_position: np.ndarray,
		armed: bool,
		guided: bool,
	) -> SensorFrame:
		frame = SensorFrame(
			time_s=float(time_s),
			local_position_ned=np.array(local_position, dtype=float),
			local_velocity_ned=np.array(local_velocity, dtype=float),
			global_position=np.array(global_position, dtype=float),
			armed=bool(armed),
			guided=bool(guided),
		)
		self.last_frame = frame
		return frame
```

---

### 2.5 `perception.py` — Perception layer (detection + localization estimates)

Purpose: produce a minimal world estimate from sensor frames.

For this MVP, perception can stay lightweight (for example: stable altitude estimate, drift estimate, and a placeholder obstacle list).

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np

from .sensing import SensorFrame


@dataclass(frozen=True)
class Obstacle:
	north: float
	east: float
	radius_m: float


@dataclass(frozen=True)
class PerceptionState:
	est_altitude_m: float
	drift_xy_mps: float
	obstacles: List[Obstacle]


@dataclass
class PerceptionLayer:
	def update(self, frame: SensorFrame) -> PerceptionState:
		vn, ve = frame.local_velocity_ned[0], frame.local_velocity_ned[1]
		drift = float(np.linalg.norm(np.array([vn, ve], dtype=float)))
		altitude = float(-frame.local_position_ned[2])
		return PerceptionState(
			est_altitude_m=altitude,
			drift_xy_mps=drift,
			obstacles=[],
		)
```

---

### 2.6 `control.py` — Control layer (trajectory tracking to commands)

Purpose: convert planner trajectory output into concrete command calls on the adapter.

```python
from __future__ import annotations

from dataclasses import dataclass

from .adapter import UdacidroneAdapter
from .planner import Trajectory


@dataclass
class ControlLayer:
	adapter: UdacidroneAdapter

	def track(self, trajectory: Trajectory) -> None:
		if not trajectory.points:
			return
		p = trajectory.points[-1]
		self.adapter.goto(
			north=p.north,
			east=p.east,
			altitude_m=p.altitude_m,
			heading=p.heading_rad,
		)
```

---

### 2.7 `guards.py` — state transition "if" logic

Purpose: pure checks that are easy to unit test.

```python
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import MissionConfig
from .types import Waypoint


@dataclass
class FlightGuards:
	cfg: MissionConfig

	def altitude_reached(self, local_position: np.ndarray) -> bool:
		# local_position is NED => altitude = -down
		altitude = -float(local_position[2])
		return altitude >= self.cfg.takeoff_altitude_ratio * self.cfg.target_altitude_m

	def waypoint_reached(self, local_position: np.ndarray, target: Waypoint) -> bool:
		pos_ne = np.array([local_position[0], local_position[1]], dtype=float)
		tgt_ne = np.array([target.north, target.east], dtype=float)
		d = float(np.linalg.norm(pos_ne - tgt_ne))
		return d <= self.cfg.pos_tolerance_m

	def ready_to_disarm(self, local_position: np.ndarray, local_velocity: np.ndarray) -> bool:
		near_ground = abs(float(local_position[2])) <= self.cfg.ground_down_tolerance_m
		vel_xy = np.linalg.norm(np.array([local_velocity[0], local_velocity[1]], dtype=float))
		nearly_stopped = float(vel_xy) <= self.cfg.vel_xy_tolerance_mps
		return near_ground and nearly_stopped
```

---

### 2.8 `adapter.py` — isolate UdaciDrone API calls

Purpose: the FSM should not care whether it’s talking to a simulator, real drone, or a mock.

This adapter is a thin wrapper around the underlying `Drone` object.

```python
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class DroneLike(Protocol):
	# attributes
	armed: bool
	guided: bool
	local_position: object
	local_velocity: object
	global_position: object

	# commands
	def arm(self) -> None: ...
	def disarm(self) -> None: ...
	def take_control(self) -> None: ...
	def release_control(self) -> None: ...
	def takeoff(self, target_altitude: float) -> None: ...
	def land(self) -> None: ...
	def cmd_position(self, north: float, east: float, altitude: float, heading: float) -> None: ...
	def set_home_position(self, lon: float, lat: float, alt: float) -> None: ...
	def stop(self) -> None: ...


@dataclass
class UdacidroneAdapter:
	drone: DroneLike

	def take_control_and_arm(self) -> None:
		self.drone.take_control()
		self.drone.arm()

		# Matches many working Backyard Flyer solutions: set home to current global position.
		# This helps ensure local frame behavior is consistent.
		try:
			gp = self.drone.global_position
			self.drone.set_home_position(gp[0], gp[1], gp[2])
		except Exception:
			pass

	def takeoff(self, altitude_m: float) -> None:
		self.drone.takeoff(altitude_m)

	def goto(self, north: float, east: float, altitude_m: float, heading: float) -> None:
		self.drone.cmd_position(north, east, altitude_m, heading)

	def land(self) -> None:
		self.drone.land()

	def disarm(self) -> None:
		self.drone.disarm()

	def finish(self) -> None:
		self.drone.release_control()
		self.drone.stop()
```

Design notes:

- The `Protocol` makes it easy to unit test FSM by passing a mock object.
- This adapter is where API-version differences can be handled.

---

### 2.9 `fsm.py` — the state machine class

Purpose: store state, manage waypoints, and call adapter commands.

We’ll mirror the project’s states.

```python
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional

import numpy as np

from .adapter import UdacidroneAdapter
from .control import ControlLayer
from .config import MissionConfig
from .guards import FlightGuards
from .planner import PlannerStack
from .perception import PerceptionLayer, PerceptionState
from .sensing import SensorFrame, SensorLayer
from .types import Waypoint


class FlightState(Enum):
	MANUAL = 0
	ARMING = 1
	TAKEOFF = 2
	WAYPOINT = 3
	LANDING = 4
	DISARMING = 5


@dataclass
class BackyardFlyerFSM:
	cfg: MissionConfig
	adapter: UdacidroneAdapter
	planner: PlannerStack
	control: ControlLayer
	sensors: SensorLayer
	perception: PerceptionLayer
	guards: FlightGuards

	state: FlightState = FlightState.MANUAL
	in_mission: bool = True

	target: Optional[Waypoint] = None
	waypoints: List[Waypoint] = field(default_factory=list)
	last_sensor_frame: Optional[SensorFrame] = None
	last_perception_state: Optional[PerceptionState] = None

	def on_sensor_update(
		self,
		time_s: float,
		local_position: np.ndarray,
		local_velocity: np.ndarray,
		global_position: np.ndarray,
		armed: bool,
		guided: bool,
	) -> None:
		self.last_sensor_frame = self.sensors.ingest(
			time_s=time_s,
			local_position=local_position,
			local_velocity=local_velocity,
			global_position=global_position,
			armed=armed,
			guided=guided,
		)
		self.last_perception_state = self.perception.update(self.last_sensor_frame)

	def on_state_update(self, armed: bool, guided: bool) -> None:
		if self.state == FlightState.MANUAL and self.in_mission:
			self._arming_transition()
			return

		if self.state == FlightState.ARMING and armed:
			self._takeoff_transition()
			return

		if self.state == FlightState.DISARMING and not armed:
			self._manual_transition()
			return

	def on_local_position(self, local_position: np.ndarray) -> None:
		if self.state == FlightState.TAKEOFF:
			if self.last_perception_state is not None:
				reached_takeoff = self.last_perception_state.est_altitude_m >= (
					self.cfg.takeoff_altitude_ratio * self.cfg.target_altitude_m
				)
			else:
				reached_takeoff = self.guards.altitude_reached(local_position)

			if not reached_takeoff:
				return

			if not self.waypoints:
				route = self.planner.route.build_box_route(
					origin_north=float(local_position[0]),
					origin_east=float(local_position[1]),
				)
				self.waypoints = route.waypoints
			self._waypoint_transition()
			return

		if self.state == FlightState.WAYPOINT and self.target is not None:
			if self.guards.waypoint_reached(local_position, self.target):
				if self.waypoints:
					self._waypoint_transition()
				else:
					self._landing_transition()

	def on_local_velocity(self, local_position: np.ndarray, local_velocity: np.ndarray) -> None:
		if self.state == FlightState.LANDING:
			if self.guards.ready_to_disarm(local_position, local_velocity):
				self._disarming_transition()

	# --- transitions (actions) ---

	def _arming_transition(self) -> None:
		print("arming transition")
		self.adapter.take_control_and_arm()
		self.state = FlightState.ARMING

	def _takeoff_transition(self) -> None:
		print("takeoff transition")
		self.adapter.takeoff(self.cfg.target_altitude_m)
		self.state = FlightState.TAKEOFF

	def _waypoint_transition(self) -> None:
		print("waypoint transition")
		current_target = self.waypoints.pop(0) if self.waypoints else None
		self.target = current_target
		if current_target is None:
			self._landing_transition()
			return

		if self.last_sensor_frame is not None:
			now_pos = self.last_sensor_frame.local_position_ned
			now_vel = self.last_sensor_frame.local_velocity_ned
		else:
			now_pos = np.array([current_target.north, current_target.east, -current_target.altitude_m], dtype=float)
			now_vel = np.zeros(3, dtype=float)

		traj = self.planner.plan_cycle(
			mission_phase="waypoint",
			local_position=now_pos,
			local_velocity=now_vel,
			next_waypoint=current_target,
		)
		self.control.track(traj)
		self.state = FlightState.WAYPOINT

	def _landing_transition(self) -> None:
		print("landing transition")
		self.adapter.land()
		self.state = FlightState.LANDING

	def _disarming_transition(self) -> None:
		print("disarm transition")
		self.adapter.disarm()
		self.state = FlightState.DISARMING

	def _manual_transition(self) -> None:
		print("manual transition")
		self.adapter.finish()
		self.in_mission = False
		self.state = FlightState.MANUAL
```

- `on_sensor_update()` creates the canonical Sensor -> Perception -> FSM data path.
- FSM still controls mission phases; planner/control handle motion-level decisions.
- `BackyardFlyerFSM` remains testable with fake sensor/perception/planner/control/adapter objects.

---

### 2.10 `app.py` — wiring (build objects once)

Purpose: central place to construct sensing, perception, planning, control, guards, and FSM.

```python
from __future__ import annotations

from .adapter import UdacidroneAdapter
from .control import ControlLayer
from .config import MissionConfig
from .fsm import BackyardFlyerFSM
from .guards import FlightGuards
from .planner import BehaviorPlanner, PlannerStack, Predictor, RoutePlanner, TrajectoryPlanner
from .perception import PerceptionLayer
from .sensing import SensorLayer


def build_fsm(drone) -> BackyardFlyerFSM:
	cfg = MissionConfig()
	adapter = UdacidroneAdapter(drone=drone)
	sensors = SensorLayer()
	perception = PerceptionLayer()
	planner = PlannerStack(
		route=RoutePlanner(cfg=cfg),
		predictor=Predictor(),
		behavior=BehaviorPlanner(),
		trajectory=TrajectoryPlanner(cfg=cfg),
	)
	control = ControlLayer(adapter=adapter)
	guards = FlightGuards(cfg=cfg)
	return BackyardFlyerFSM(
		cfg=cfg,
		adapter=adapter,
		planner=planner,
		control=control,
		sensors=sensors,
		perception=perception,
		guards=guards,
	)
```

---

## 3) Hooking this into the Udacity starter (`backyard_flyer.py`)

You need a `Drone` subclass for UdaciDrone callbacks, but it can be extremely thin:

### 3.1 Pattern: `BackyardFlyer(Drone)` delegates to `BackyardFlyerFSM`

In the existing `backyard_flyer.py`:

1) Build the FSM once in `__init__`.
2) In each callback, forward the updated attributes.

Pseudo-diff (illustrative):

```python
import time

from backyard_flyer.app import build_fsm

class BackyardFlyer(Drone):
	def __init__(self, connection):
		super().__init__(connection)
		self.fsm = build_fsm(self)
		... register callbacks ...

	def _update_sensor_pipeline(self):
		self.fsm.on_sensor_update(
			time_s=time.time(),
			local_position=self.local_position,
			local_velocity=self.local_velocity,
			global_position=self.global_position,
			armed=self.armed,
			guided=self.guided,
		)

	def local_position_callback(self):
		self._update_sensor_pipeline()
		self.fsm.on_local_position(self.local_position)

	def velocity_callback(self):
		self._update_sensor_pipeline()
		self.fsm.on_local_velocity(self.local_position, self.local_velocity)

	def state_callback(self):
		self._update_sensor_pipeline()
		self.fsm.on_state_update(self.armed, self.guided)
```

At this point, the existing `start()` method and logging behavior can remain unchanged.

---

## 4) Unit tests you can write immediately

### 4.1 `test_planning.py`

```python
from backyard_flyer.config import MissionConfig
from backyard_flyer.planner import PlannerStack, Predictor, RoutePlanner, TrajectoryPlanner, BehaviorPlanner


def test_route_planner_returns_4_waypoints():
	cfg = MissionConfig(box_size_m=10.0, target_altitude_m=3.0)
	route = RoutePlanner(cfg)
	wps = route.build_box_route(origin_north=0.0, origin_east=0.0).waypoints
	assert len(wps) == 4
	assert wps[-1].north == 0.0
	assert wps[-1].east == 0.0
	assert wps[0].altitude_m == 3.0


def test_planner_stack_tracks_route_when_waypoint_exists():
	cfg = MissionConfig()
	route = RoutePlanner(cfg=cfg)
	planner = PlannerStack(
		route=route,
		predictor=Predictor(),
		behavior=BehaviorPlanner(),
		trajectory=TrajectoryPlanner(cfg=cfg),
	)
	next_waypoint = route.build_box_route(origin_north=0.0, origin_east=0.0).waypoints[0]
	traj = planner.plan_cycle(
		mission_phase="waypoint",
		local_position=[0.0, 0.0, -3.0],
		local_velocity=[0.0, 0.0, 0.0],
		next_waypoint=next_waypoint,
	)
	assert len(traj.points) == 1
```

### 4.2 `test_guards.py`

```python
import numpy as np

from backyard_flyer.config import MissionConfig
from backyard_flyer.guards import FlightGuards
from backyard_flyer.types import Waypoint


def test_altitude_reached_uses_ned_down_sign():
	cfg = MissionConfig(target_altitude_m=3.0, takeoff_altitude_ratio=0.95)
	guards = FlightGuards(cfg)
	# at altitude 3m => down = -3
	assert guards.altitude_reached(np.array([0.0, 0.0, -3.0]))


def test_waypoint_reached_xy_only():
	cfg = MissionConfig(pos_tolerance_m=0.5)
	guards = FlightGuards(cfg)
	target = Waypoint(10.0, 0.0, 3.0)
	assert guards.waypoint_reached(np.array([10.2, 0.1, -3.0]), target)
```

---

### 4.3 Optional: `test_sensor_perception.py`

```python
import numpy as np

from backyard_flyer.perception import PerceptionLayer
from backyard_flyer.sensing import SensorLayer


def test_sensor_to_perception_pipeline_smoke():
	sensors = SensorLayer()
	perception = PerceptionLayer()
	frame = sensors.ingest(
		time_s=1.0,
		local_position=np.array([0.0, 0.0, -3.0]),
		local_velocity=np.array([0.1, 0.2, 0.0]),
		global_position=np.array([0.0, 0.0, 0.0]),
		armed=True,
		guided=True,
	)
	state = perception.update(frame)
	assert state.est_altitude_m == 3.0
	assert state.drift_xy_mps > 0.0
```

---

## 5) Validation checklist (how you know your modular version works)

### 5.1 Logic validation (fast)

- Run unit tests for planning + guards (+ optional sensor/perception smoke tests).
- Sanity-check that altitude uses NED sign correctly.
- Confirm each callback updates the sensor pipeline before transition checks.

### 5.2 Simulator validation (end-to-end)

- Start the Unity simulator.
- Run your script (`python backyard_flyer.py`).
- Observe state transitions printed once per transition:
  - arming → takeoff → waypoint (x4 corners) → landing → disarm → manual
- Confirm the vehicle flies a square path and lands.

---

## 6) How this supports the MiNiFi/NiFi evolution (without changing control)

Once modular, telemetry becomes a separate concern:

- Sensor and Perception outputs become stable event schemas for streaming.
- Add a `telemetry/logger.py` that writes JSONL events.
- MiNiFi can tail those files and forward to a central NiFi flow.

Critically:

- The FSM and Control remain deterministic and local.
- Dataflow (MiNiFi/NiFi) can fail without impacting flight safety.

---

## 7) If you want an even simpler modular variant

If this feels like “too many files” for learning, collapse it to 3 files:

1) `mission_config.py`
2) `fsm.py` (including sensor/perception/planner/control + guards)
3) `backyard_flyer.py` (thin Drone wrapper)

Then split modules out only after your first successful sim run.

