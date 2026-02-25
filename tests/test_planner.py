from backyard_flyer.config import MissionConfig
from backyard_flyer.planner import BoxPlanner

def test_box_planner_returns_4_waypoints():
    cfg = MissionConfig(
        box_size_m=10.0, target_altitude_m=3.0
    )

    planner = BoxPlanner(cfg)

    wps = planner.build_box(origin_north=0.0, origin_east=0.0)

    assert len(wps) = 4
    assert wps[-1].north == 0.0
    assert wps[-1].east == 0.0
    assert wps[0].down == -3.0
