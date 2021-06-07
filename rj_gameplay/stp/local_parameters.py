"""
Usage: in the global scope of a module, at initialization time, construct a Param.
```python
from stp.local_parameters import Param
my_parameter = Param("my_param_name", 1.0, Param.PARAMETER_DOUBLE)
```
Call `register_parameters` in gameplay init. Then, use the parameters:
```
do_something(my_parameter.value)
```
"""

from typing import List, Optional

from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult, ParameterType, ParameterDescriptor

parameters = {}
param_node: Optional[Node] = None


class Param:
    PARAMETER_NOT_SET = ParameterType.PARAMETER_NOT_SET
    PARAMETER_BOOL = ParameterType.PARAMETER_BOOL
    PARAMETER_INTEGER = ParameterType.PARAMETER_INTEGER
    PARAMETER_DOUBLE = ParameterType.PARAMETER_DOUBLE
    PARAMETER_BYTE_ARRAY = ParameterType.PARAMETER_BYTE_ARRAY
    PARAMETER_BOOL_ARRAY = ParameterType.PARAMETER_BOOL_ARRAY
    PARAMETER_INTEGER_ARRAY = ParameterType.PARAMETER_INTEGER_ARRAY
    PARAMETER_DOUBLE_ARRAY = ParameterType.PARAMETER_DOUBLE_ARRAY

    def __init__(self, name: str, default_value, param_type: ParameterType = None, description: str = ''):
        global parameters, param_node

        self.name = name
        if name not in parameters:
            parameters[name] = (default_value, param_type, description)

        if param_node is not None:
            register_parameter(param_node, name, default_value, param_type, description)

    @property
    def value(self):
        global parameters

        return parameters[self.name][0]


def update_params(params: List[Parameter]) -> SetParametersResult:
    """
    Update a set of parameters with new values.
    :param params: A list of ROS Parameter changes
    """
    for param in params:
        parameters[param.name] = (param.value, parameters[param.name][1], parameters[param.name][2])

    return SetParametersResult(successful=True)


def register_parameters(node: Node):
    """
    Register all parameters. This should be called after module initialization.

    :param node: The gameplay node
    """
    global param_node, parameters

    assert param_node is None

    for param_name in parameters:
        value, param_type, description = parameters[param_name]
        register_parameter(node, param_name, value, param_type, description)
    node.add_on_set_parameters_callback(update_params)

    param_node = node


def register_parameter(node: Node, param_name: str, value, param_type: ParameterType, description: Optional[str]):
    global parameters

    descriptor = ParameterDescriptor(name=param_name, type=param_type, description=description or '')
    if not node.has_parameter(param_name):
        node.declare_parameter(param_name, value, descriptor)
    parameters[param_name] = node.get_parameter(param_name).value, parameters[param_name][1], parameters[param_name][2]
