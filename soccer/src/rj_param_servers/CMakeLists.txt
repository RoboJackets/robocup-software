# ---- config_server_param_provider ----
set(CONFIG_SERVER_PARAM_PROVIDER_SRC config_server_param_provider.cpp)
target_sources(config_server_param_provider PRIVATE ${CONFIG_SERVER_PARAM_PROVIDER_SRC})

# ---- control_param_provider ----
set(CONTROL_PARAM_PROVIDER_SRC control_param_provider.cpp)
target_sources(control_param_provider PRIVATE ${CONTROL_PARAM_PROVIDER_SRC})

# ---- gameplay_node_param_provider ----
set(GAMEPLAY_NODE_PARAM_PROVIDER_SRC gameplay_node_param_provider.cpp)
target_sources(gameplay_node_param_provider PRIVATE ${GAMEPLAY_NODE_PARAM_PROVIDER_SRC})

# ---- global_param_provider ----
set(GLOBAL_PARAM_PROVIDER_SRC global_param_provider.cpp)
target_sources(global_param_provider PRIVATE ${GLOBAL_PARAM_PROVIDER_SRC})

# ---- internal_referee_param_provider ----
set(INTERNAL_REFEREE_PARAM_PROVIDER_SRC internal_referee_param_provider.cpp)
target_sources(internal_referee_param_provider PRIVATE ${INTERNAL_REFEREE_PARAM_PROVIDER_SRC})

# ---- planner_param_provider ----
set(PLANNER_PARAM_PROVIDER_SRC planner_param_provider.cpp)
target_sources(planner_param_provider PRIVATE ${PLANNER_PARAM_PROVIDER_SRC})

# ---- vision_receiver_param_provider ----
set(VISION_RECEIVER_PARAM_PROVIDER_SRC vision_receiver_param_provider.cpp)
target_sources(vision_receiver_param_provider PRIVATE ${PLANNER_PARAM_PROVIDER_SRC})

# ---- radio_param_provider ----
set(RADIO_PARAM_PROVIDER_SRC radio_param_provider.cpp)
target_sources(radio_param_provider PRIVATE ${RADIO_PARAM_PROVIDER_SRC})

# ---- vision_filter_param_provider ----
set(VISION_FILTER_PARAM_PROVIDER_SRC vision_filter_param_provider.cpp)
target_sources(vision_filter_param_provider PRIVATE ${VISION_FILTER_PARAM_PROVIDER_SRC})
