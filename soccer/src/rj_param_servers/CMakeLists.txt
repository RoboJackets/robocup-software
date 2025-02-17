set(CONFIG_SERVER_PARAM_PROVIDER_SRC config_server_param_provider.cpp)

# ---- config_server_param_provider ----
target_sources(config_server_param_provider PRIVATE ${CONFIG_SERVER_PARAM_PROVIDER_SRC})

set(CONTROL_PARAM_PROVIDER_SRC control_param_provider.cpp)

# ---- control_param_provider ----
target_sources(control_param_provider PRIVATE ${CONTROL_PARAM_PROVIDER_SRC})
