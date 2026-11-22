#!/bin/bash
# Builds build/librmf_map_remap.so (generated artifact, not committed). Needs: gcc and /opt/ros/humble (headers only).
set -eu; HERE=$(cd "$(dirname "$0")" && pwd); mkdir -p "$HERE/build"
I=/opt/ros/humble/include
gcc -shared -fPIC -O2 -Wall -Wextra -o "$HERE/build/librmf_map_remap.so" "$HERE/rmf_map_remap.c" -I$I/rcl -I$I/rcutils -I$I/rmw -I$I/rcl_yaml_param_parser -I$I/rosidl_runtime_c -I$I/rosidl_typesupport_interface -I$I/rcl_logging_interface -I$I/type_description_interfaces -ldl
sha256sum "$HERE/build/librmf_map_remap.so"
