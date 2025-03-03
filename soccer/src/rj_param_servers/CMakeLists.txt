set(CONFIG_SERVER_PARAM_PROVIDER_SRC config_server_param_provider.cpp)

# ---- config_server_param_provider ----
target_sources(config_server_param_provider PRIVATE ${CONFIG_SERVER_PARAM_PROVIDER_SRC})

set(CONTROL_PARAM_PROVIDER_SRC control_param_provider.cpp)

# ---- control_param_provider ----
target_sources(control_param_provider PRIVATE ${CONTROL_PARAM_PROVIDER_SRC})

set(GAMEPLAY_NODE_PARAM_PROVIDER_SRC gameplay_node_param_provider.cpp)

# ---- gameplay_node_param_provider ----
target_sources(gameplay_node_param_provider PRIVATE ${GAMEPLAY_NODE_PARAM_PROVIDER_SRC})

set(GLOBAL_PARAM_PROVIDER_SRC global_param_provider.cpp)

# ---- global_param_provider ----
target_sources(global_param_provider PRIVATE ${GLOBAL_PARAM_PROVIDER_SRC})

set(INTERNAL_REFEREE_PARAM_PROVIDER_SRC internal_referee_param_provider.cpp)

# ---- internal_referee_param_provider ----
target_sources(internal_referee_param_provider PRIVATE ${INTERNAL_REFEREE_PARAM_PROVIDER_SRC})
