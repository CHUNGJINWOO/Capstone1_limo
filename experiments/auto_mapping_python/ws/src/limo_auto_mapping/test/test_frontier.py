"""Python 후보 선택의 지도 경계·비용·제외 조건을 검사한다. 실물 주행 검사가 아니다."""
import math
import pytest
from limo_auto_mapping.frontier import blacklisted, choose_frontiers
from limo_auto_mapping.policy import cost, world
from test_policy import grid


def test_goal_is_free_cell_next_to_unknown_and_never_centroid_in_wall():
    raw = grid([[100]*5, [100, 0, 0, 0, 100], [100, -1, -1, -1, 100]])
    candidates = choose_frontiers(raw, world(raw, 6))
    assert len(candidates) == 1
    assert candidates[0].size == 3
    assert cost(raw, candidates[0].point) == 0


def test_costmap_inflation_hides_frontier_until_reduced():
    inflated = grid([[100]*5, [100, 0, 50, 50, 100], [100, 100, -1, -1, 100]])
    assert choose_frontiers(inflated, world(inflated, 6)) == []
    inflated.data[7] = inflated.data[8] = 0
    assert len(choose_frontiers(inflated, world(inflated, 6))) == 1


def test_disconnected_room_and_row_wrap_cannot_create_reachable_goal():
    raw = grid([[100, 100, 100, 100, 0], [0, -1, 100, 100, 100]])
    assert choose_frontiers(raw, world(raw, 4)) == []


def test_small_boundary_and_obstacles_do_not_become_goals():
    raw = grid([[100]*3, [100, 0, -1], [100]*3], resolution=.05)
    assert choose_frontiers(raw, world(raw, 4), minimum_size=.2) == []


def test_inflated_robot_cell_can_find_connected_free_seed():
    raw = grid([[100]*5, [100, 40, 0, 0, -1], [100]*5])
    assert choose_frontiers(raw, world(raw, 6))[0].point == world(raw, 8)


@pytest.mark.parametrize('value', [-1, 99, 100])
def test_invalid_robot_cell_fails_instead_of_claiming_completion(value):
    raw = grid([[value, 0, -1]])
    with pytest.raises(ValueError):
        choose_frontiers(raw, world(raw, 0))


def test_rotated_map_returns_valid_world_goal():
    raw = grid([[0, 0, -1]], angle=math.pi/2)
    assert cost(raw, choose_frontiers(raw, world(raw, 0))[0].point) == 0


def test_exclusion_radius_prevents_repeated_goal():
    assert blacklisted((1.0, 1.0), [(1.1, 1.0)], .2)
    assert not blacklisted((2.0, 1.0), [(1.1, 1.0)], .2)
