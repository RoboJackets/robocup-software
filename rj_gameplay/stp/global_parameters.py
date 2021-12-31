import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent, ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters


class ParamTreeNode:
    def __init__(self):
        pass


class GlobalParameterClient:
    """
    A client for the C++-side global parameter server. Allows us to share constants between C++ nodes and gameplay.

    After creating this class, global parameters will be available as globals in the global_parameters module.
    """

    def __init__(self, node: Node, global_param_server: str):
        """
        Initialize the global parameter client. Do this before running anything that might use global parameters.
        :param node: the gameplay node.
        :param global_param_server: fully-qualified name of the global parameter server, i.e. /global_parameter_server
        """
        self.global_param_server = global_param_server

        list_client = node.create_client(
            ListParameters, f"{global_param_server}/list_parameters"
        )
        get_client = node.create_client(
            GetParameters, f"{global_param_server}/get_parameters"
        )
        #sub = node.create_subscription(
            #ParameterEvent, f"/parameter_events", self.update_parameters, 10
        #)

        list_parameter_request = ListParameters.Request()
        list_future = list_client.call_async(list_parameter_request)

        rclpy.spin_once(node, timeout_sec=0.1)

        # TODO(Kyle): figure out why service calls are so flaky on startup
        rclpy.spin_until_future_complete(node, list_future, timeout_sec=0.5)
        while not list_future.done():
            list_future.cancel()
            list_future = list_client.call_async(list_parameter_request)
            print("Waiting for ListParameters")
            rclpy.spin_until_future_complete(node, list_future, timeout_sec=0.5)

        params_names = list_future.result().result.names
        get_parameter_request = GetParameters.Request(names=params_names)
        get_future = get_client.call_async(get_parameter_request)

        # TODO(Kyle): figure out why service calls are so flaky on startup
        rclpy.spin_until_future_complete(node, get_future, timeout_sec=0.5)
        while not get_future.done():
            get_future.cancel()
            get_future = get_client.call_async(get_parameter_request)
            print("Waiting for GetParameters")
            rclpy.spin_until_future_complete(node, get_future, timeout_sec=0.5)

        params_values = get_future.result().values
        for param, value_ros in zip(params_names, params_values):
            self.set_param(param, value_ros)

    def update_parameters(self, event: ParameterEvent):
        if event.node == self.global_param_server:
            for param in event.changed_parameters + event.new_parameters:
                self.set_param(param.name, param.value)

    def set_param(self, name, value_ros):
        value = None
        if value_ros.type == ParameterType.PARAMETER_NOT_SET:
            pass
        elif value_ros.type == ParameterType.PARAMETER_BOOL:
            value = value_ros.bool_value
        elif value_ros.type == ParameterType.PARAMETER_INTEGER:
            value = value_ros.integer_value
        elif value_ros.type == ParameterType.PARAMETER_DOUBLE:
            value = value_ros.double_value
        elif value_ros.type == ParameterType.PARAMETER_BYTE_ARRAY:
            value = value_ros.byte_array_value
        elif value_ros.type == ParameterType.PARAMETER_BOOL_ARRAY:
            value = value_ros.bool_array_value
        elif value_ros.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            value = value_ros.integer_array_value
        elif value_ros.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            value = value_ros.double_array_value
        names = name.split(".")
        tree = globals()
        for prefix in names[:-1]:
            if prefix not in tree:
                tree[prefix] = ParamTreeNode()
            tree = tree[prefix].__dict__
        tree[names[-1]] = value


if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("_test_gps")
    client = GlobalParameterClient(node, "/global_parameter_server")
    rclpy.spin(node)
    rclpy.shutdown()
