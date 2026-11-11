from types import SimpleNamespace as NS
import math
import pytest
from limo_auto_mapping.policy import (
    cost, frontiers, index, suppressed_candidates, validate_radii, world,
)


def grid(rows, resolution=1.0, angle=0.0):
    return NS(header=NS(frame_id='map'), data=sum(rows, []), info=NS(
        width=len(rows[0]), height=len(rows), resolution=resolution,
        origin=NS(position=NS(x=-2.0, y=3.0), orientation=NS(
            x=0.0, y=0.0, z=math.sin(angle/2), w=math.cos(angle/2)))))


def test_inflation_evidence_excludes_lethal_inscribed_unknown_and_outside():
    raw = grid([[100]*6, [100, 0, 0, 0, 0, 100], [100, -1, -1, -1, -1, 100]])
    inflated = grid([[100]*6, [100, 40, 99, 100, -1, 100], [100]*6])
    robot = world(raw, 7)
    assert suppressed_candidates(raw, inflated, robot) == [robot]
    assert cost(inflated, (100, 100)) is None


def test_disconnected_unknown_does_not_trigger_retry():
    raw = grid([[100]*5, [100, 0, 100, 0, 100], [100, 100, 100, -1, 100]])
    assert frontiers(raw, world(raw, 6)) == []


def test_free_frontier_not_inflation_evidence():
    raw = grid([[100]*3, [100, 0, -1], [100]*3])
    assert len(frontiers(raw, world(raw, 4))) == 1
    assert suppressed_candidates(raw, raw, world(raw, 4)) == []


def test_grid_coordinates_support_rotated_map_and_floor_outside():
    raw = grid([[0, 0], [0, 0]], angle=math.pi/2)
    for cell in range(4):
        assert index(raw, *world(raw, cell)) == cell
    assert index(grid([[0]]), -2.01, 3.5) is None


def test_invalid_map_and_frame_fail_closed():
    raw = grid([[0, -1]])
    other = grid([[40, -1]])
    other.header.frame_id = 'odom'
    with pytest.raises(ValueError):
        suppressed_candidates(raw, other, world(raw, 0))
    raw.data = []
    with pytest.raises(ValueError):
        frontiers(raw, (0, 0))


@pytest.mark.parametrize('original,reduced,radius,clearance', [
    ([.3, .3], .18, .19, .05), ([.3, .24], .24, .19, .05),
    ([.3, .3], float('nan'), .19, .05), ([.3, .3], .24, -.1, .05),
    ([.3, .3], .24, .19, 0),
])
def test_reduction_bounds_reject_unsafe_or_non_reduction(original, reduced, radius, clearance):
    with pytest.raises(ValueError):
        validate_radii(original, reduced, radius, clearance)


def test_reduction_floor():
    validate_radii([.30, .30], .24, .19, .05)
