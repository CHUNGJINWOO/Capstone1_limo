/* rmf_map_remap.c - LD_PRELOAD shim: keep nav_msgs/OccupancyGrid on /map by moving ONLY
 * rmf_building_map_msgs/msg/BuildingMap publishers/subscriptions from "/map" to "/rmf/building_map".
 *
 * Why: the stock Open-RMF Gazebo slotcar plugin (libslotcar_common.so, binary from the rmf_robot_sim debs) subscribes to the
 * absolute topic "/map" with BuildingMap and cannot be remapped from a launch file (the plugin calls rclcpp::init(0, nullptr)).
 * SLAM Toolbox / Nav2 map_server publish nav_msgs/OccupancyGrid on /map, so /map gets two types and
 * `ros2 bag record /map` fails ("Topic '/map' has more than one type associated").
 *
 * Scope: only calls whose message type is exactly rmf_building_map_msgs::msg::BuildingMap AND whose topic is "/map" or "map" are
 * changed. Every other publisher/subscription (including OccupancyGrid on /map) passes through unchanged. If the type cannot be
 * determined the call passes through unchanged. Override target with env RMF_BUILDING_MAP_TOPIC (absolute name).
 * Build: see build.sh. Use: LD_PRELOAD=<abs path>/librmf_map_remap.so <rmf/gazebo process>. No file of the RMF install is modified. */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <rcl/rcl.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

/* first two members of rosidl_typesupport_introspection_{c,cpp}::MessageMembers */
typedef struct { const char * message_namespace_; const char * message_name_; } members_head_t;

static int is_building_map(const rosidl_message_type_support_t * ts)
{
  if (!ts || !ts->typesupport_identifier) {return 0;}
  const rosidl_message_type_support_t * h = NULL;
  const char * id = ts->typesupport_identifier;
  if (strcmp(id, "rosidl_typesupport_introspection_cpp") == 0 || strcmp(id, "rosidl_typesupport_introspection_c") == 0) {
    h = ts;
  } else if (ts->func) {
    /* the generic typesupport handles (rclpy: _c, rclcpp: _cpp) resolve to the matching introspection handle */
    const char * want = NULL;
    if (strcmp(id, "rosidl_typesupport_c") == 0) {want = "rosidl_typesupport_introspection_c";}
    else if (strcmp(id, "rosidl_typesupport_cpp") == 0) {want = "rosidl_typesupport_introspection_cpp";}
    if (!want) {return 0;}
    h = ts->func(ts, want);
  }
  if (!h || !h->data) {return 0;}
  const members_head_t * m = (const members_head_t *)h->data;
  if (!m->message_namespace_ || !m->message_name_) {return 0;}
  /* cpp introspection namespace: "rmf_building_map_msgs::msg"; c introspection: "rmf_building_map_msgs__msg" */
  return strcmp(m->message_name_, "BuildingMap") == 0 && strncmp(m->message_namespace_, "rmf_building_map_msgs", 21) == 0;
}

static const char * target(const char * topic, const rosidl_message_type_support_t * ts)
{
  if (!topic || (strcmp(topic, "/map") != 0 && strcmp(topic, "map") != 0)) {return topic;}
  if (!is_building_map(ts)) {return topic;}
  const char * t = getenv("RMF_BUILDING_MAP_TOPIC");
  if (!t || t[0] != '/') {t = "/rmf/building_map";}
  static int logged = 0;
  if (!logged) {logged = 1; fprintf(stderr, "[rmf_map_remap] BuildingMap %s -> %s (OccupancyGrid /map untouched)\n", topic, t);}
  return t;
}

/* dlsym(RTLD_NEXT) does not see librcl when it was dlopen()ed RTLD_LOCAL by an extension module (rclpy); fall back to the loaded librcl. */
static void * find_real(const char * name)
{
  void * f = dlsym(RTLD_NEXT, name);
  if (!f) {
    void * h = dlopen("librcl.so", RTLD_LAZY | RTLD_NOLOAD);
    if (h) {f = dlsym(h, name);}
  }
  return f;
}

rcl_ret_t rcl_subscription_init(
  rcl_subscription_t * subscription, const rcl_node_t * node, const rosidl_message_type_support_t * type_support,
  const char * topic_name, const rcl_subscription_options_t * options)
{
  static rcl_ret_t (* real)(rcl_subscription_t *, const rcl_node_t *, const rosidl_message_type_support_t *, const char *,
    const rcl_subscription_options_t *) = NULL;
  if (!real) {real = find_real("rcl_subscription_init");}
  if (!real) {return RCL_RET_ERROR;}
  return real(subscription, node, type_support, target(topic_name, type_support), options);
}

rcl_ret_t rcl_publisher_init(
  rcl_publisher_t * publisher, const rcl_node_t * node, const rosidl_message_type_support_t * type_support,
  const char * topic_name, const rcl_publisher_options_t * options)
{
  static rcl_ret_t (* real)(rcl_publisher_t *, const rcl_node_t *, const rosidl_message_type_support_t *, const char *,
    const rcl_publisher_options_t *) = NULL;
  if (!real) {real = find_real("rcl_publisher_init");}
  if (!real) {return RCL_RET_ERROR;}
  return real(publisher, node, type_support, target(topic_name, type_support), options);
}
