"""Mapping processes have one owner, the opt-in Agent session."""


def worker_parameters(mapping, bag):
    # G2: the v5 Agent's Mapping Worker parameter is web_mapping_enabled (origin/main: isolated_ros_mapping); both are passed,
    # the v5 Agent ignores the unknown one. Bag keeps origin/main's isolated_ros_bag.
    return {'isolated_ros_mapping': mapping, 'web_mapping_enabled': mapping, 'isolated_ros_bag': bag}
