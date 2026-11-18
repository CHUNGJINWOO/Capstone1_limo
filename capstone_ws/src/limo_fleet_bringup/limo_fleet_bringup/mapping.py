"""Mapping processes have one owner, the opt-in Agent session."""


def worker_parameters(mapping, bag):
    return {'isolated_ros_mapping': mapping, 'isolated_ros_bag': bag}
