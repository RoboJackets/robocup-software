#include <config_client/config_client_node.h>

namespace config_client {
ConfigClientNode::ConfigClientNode(const std::string& name) : Node{name}, config_client_(this) {}
}  // namespace config_client
