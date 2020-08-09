#include <config_client/config_client_node.hpp>

namespace config_client {
ConfigClientNode::ConfigClientNode(const std::string& name) : Node{name}, config_client_(this) {}
}  // namespace config_client
