#include <config_client/async_config_client.h>

namespace config_client {
AsyncConfigClient::AsyncConfigClient(const std::string& name) {
    config_client_ = std::make_shared<ConfigClientNode>(name);
    executor_.add_node(config_client_);

    worker_thread_ = std::thread([this]() { executor_.spin(); });

    if (config_client_ == nullptr) {
        throw std::runtime_error("omg why is config_client_ nullptr?!");
    }
}

ConfigClientNode& AsyncConfigClient::config_client_node() {
    return *config_client_;
}

}  // namespace config_client