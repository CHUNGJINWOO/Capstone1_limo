import math
from types import SimpleNamespace as NS
from audit_metrics import center_cost, contact_episodes, trajectory_metrics, box_cost_summary


def test_cost_uses_floor_not_round():
    grid = NS(info=NS(width=3, height=1, resolution=1.,
                     origin=NS(position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.))),
              data=[0, 99, 100])
    assert center_cost(grid, (.75, .1)) == 0
    assert center_cost(grid, (-.01, .1)) is None
    grid.info.origin.orientation = NS(x=0., y=0., z=math.sin(math.pi/4), w=math.cos(math.pi/4))
    assert center_cost(grid, (-.1, .75)) == 0


def test_contact_samples_are_deduplicated_and_split_by_time_gap():
    samples = [{'stamp_ns': n, 'pair': ['limo::body', 'glass::wall']}
               for n in (0, 0, 30_000_000, 60_000_000, 500_000_000)]
    episodes = contact_episodes(samples)
    assert len(episodes) == 2
    assert episodes[0]['duration_sec'] == .06


def test_world_trajectory_ignores_repeated_timestamp():
    samples = [{'stamp_ns': 1, 'x': -2.2, 'y': 0.},
               {'stamp_ns': 1, 'x': -2.2, 'y': 0.},
               {'stamp_ns': 2, 'x': -1.2, 'y': 0.},
               {'stamp_ns': 3, 'x': -2.2, 'y': 0.}]
    result = trajectory_metrics(samples)
    assert result['samples'] == 3
    assert result['displacement_m'] == 0.
    assert math.isclose(result['cumulative_distance_m'], 2.)


def test_map_and_world_coordinates_are_not_compared_directly():
    from audit_metrics import map_point_to_world
    world = map_point_to_world((.8, 0.), (0., 0., 0.), (-2.2, 0., 0.))
    assert math.isclose(world[0], -1.4)
    assert math.isclose(world[1], 0.)
    rotated = map_point_to_world((1., 0.), (0., 0., 0.), (0., -1., math.pi/2))
    assert math.isclose(rotated[0], 0., abs_tol=1e-9)
    assert math.isclose(rotated[1], 0., abs_tol=1e-9)


def test_visible_box_surface_counts_when_center_cell_is_free():
    grid = NS(info=NS(width=3, height=3, resolution=.1,
                     origin=NS(position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.))),
              data=[0, 100, 0, 0, 0, 0, 0, 0, 0])
    assert center_cost(grid, (.15, .15)) == 0
    result = box_cost_summary(grid, (-2.05, .15), (.3, .3), (0., 0., 0.), (-2.2, 0., 0.))
    assert result == {'known_cells': 9, 'maximum_cost': 100,
                      'inscribed_or_obstacle_cells': 1, 'lethal_cells': 1}


def test_box_region_uses_map_origin_and_world_rotation():
    grid = NS(info=NS(width=1, height=1, resolution=.1,
                     origin=NS(position=NS(x=1., y=0.), orientation=NS(x=0., y=0., z=0., w=1.))),
              data=[99])
    result = box_cost_summary(grid, (-.05, 1.05), (.1, .1), (0., 0., 0.), (0., 0., math.pi/2))
    assert result['inscribed_or_obstacle_cells'] == 1
    empty = box_cost_summary(grid, (5., 5.), (.1, .1), (0., 0., 0.), (0., 0., math.pi/2))
    assert empty['maximum_cost'] is None
