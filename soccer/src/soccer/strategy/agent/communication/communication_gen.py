import os
import re

def convert_msg_to_hpp_include(msg):
    words = re.findall("[A-Z][^A-Z]*", msg[:-4])
    words = list(map(lambda x: x.lower(), words))
    return "#include \"rj_msgs/msg/" + "_".join(words) + ".hpp\""

def map_message_type_to_cpp_type(msg_type):
    msg_type = msg_type.strip()
    if "int" in msg_type:
        space_loc = msg_type.find(" ")
        return msg_type[:space_loc] + "_t" + msg_type[space_loc:] + ";"
    elif "string" in msg_type:
        return "std::" + msg_type + ";"
    elif "float64" in msg_type:
        space_loc = msg_type.find(" ")
        return "double " + msg_type[space_loc + 1:] + ";"
    return msg_type + ";"

def convert_individual_hpp(msg, path, msg_type):
    msgName = msg[:-4] #Stripping .msg from the end
    hpp = "#pragma once \n\n#include <mutex>\n#include <string>\n#include <variant>\n#include <vector>\n\n#include <rj_common/time.hpp>\n#include <rj_convert/ros_convert.hpp>\n"
    hpp += convert_msg_to_hpp_include(msg)
    hpp += "\n\n"
    hpp += "namespace strategy::communication {\n\n"
    hpp += "struct " + msgName + " {\n"
    with open(path + "/" + msg) as f:
        for line in f:
            if "#" not in line:
                hpp += "\t"
                hpp += map_message_type_to_cpp_type(line)
                hpp += "\n"
    hpp += "};\n\n"
    hpp += "bool operator==(const " + msgName + "& a, const " + msgName + "& b);\n"
    hpp += "void generate_uid(" + msgName + "& " + msg_type + ");\n\n"
    hpp += "}\n\n"
    hpp += "namespace rj_convert {\n\n"
    hpp += "template <>\n"
    hpp += "struct RosConverter<strategy::communication::" + msgName + ", rj_msgs::msg::" + msgName + "> {\n"
    hpp += "\tstatic rj_msgs::msg::" + msgName + " to_ros(const strategy::communication::" + msgName + "& from) {\n"
    hpp += "\t\trj_msgs::msg::" + msgName + " result;\n"
    with open(path + "/" + msg) as f:
        for line in f:
            if "#" not in line:
                varName = line.split(" ")[1].strip()
                hpp += "\t\tresult." + varName + " = from." + varName + ";\n"
    hpp += "\t\treturn result;\n"
    hpp += "\t}\n\n"
    hpp += "\tstatic strategy::communication::" + msgName + " from_ros(const rj_msgs::msg::" + msgName + "& from) {\n"
    hpp += "\t\treturn strategy::communication::" + msgName + "{\n"
    with open(path + "/" + msg) as f:
        for line in f:
            if "#" not in line:
                varName = line.split(" ")[1].strip()
                hpp += "\t\t\tfrom." + varName + ",\n"
    hpp += "\t\t};\n"
    hpp += "\t}\n\n"
    hpp += "};\n\n"
    hpp += "ASSOCIATE_CPP_ROS(strategy::communication::" + msgName + ", rj_msgs::msg::" + msgName + ");\n\n"
    hpp += "}"
    return hpp

def convert_cpp(requests, responses, hpp_names):
    cpp = "#include \"communication.hpp\"\n"
    cpp += "\nnamespace strategy::communication {\n\n"
    cpp += "std::mutex request_uid_mutex;\n"
    cpp += "u_int32_t request_uid = 0;\n\n"
    cpp += "std::mutex response_uid_mutex;\n"
    cpp += "u_int32_t response_uid = 0;\n\n"

    for request in requests:
        msgName = request[:-4] #Stripping .msg from the end
        cpp += "bool operator==(const " + msgName + "& a, const " + msgName + "& b) {\n"
        cpp += "\treturn a.request_uid == b.request_uid;\n"
        cpp += "}\n\n"

    for response in responses:
        msgName = response[:-4] #Stripping .msg from the end
        cpp += "bool operator==(const " + msgName + "& a, const " + msgName + "& b) {\n"
        cpp += "\treturn a.response_uid == b.response_uid;\n"
        cpp += "}\n\n"

    cpp += "bool operator==(const AgentRequest& a, const AgentRequest& b) {\n"
    cpp += "\treturn (a.request == b.request) && (a.response == b.response);\n"
    cpp += "}\n\n"

    for request in requests:
        msgName = request[:-4] #Stripping .msg from the end
        cpp += "void generate_uid(" + msgName + "& request) {\n"
        cpp += "\trequest_uid_mutex.lock();\n"
        cpp += "\trequest.request_uid = request_uid;\n"
        cpp += "\trequest_uid++;\n"
        cpp += "\trequest_uid_mutex.unlock();\n"
        cpp += "}\n\n"

    for response in responses:
        msgName = response[:-4] #Stripping .msg from the end
        cpp += "void generate_uid(" + msgName + "& response) {\n"
        cpp += "\tresponse_uid_mutex.lock();\n"
        cpp += "\tresponse.response_uid = response_uid;\n"
        cpp += "\tresponse_uid++;\n"
        cpp += "\tresponse_uid_mutex.unlock();\n"
        cpp += "}\n\n"

    cpp += "}"
    return cpp

def create_cpp_file(requests, responses, hpp_names):
    cpp = convert_cpp(requests, responses, hpp_names)
    with open("communication.cpp", "w") as f:
        f.write(cpp)

def create_hpp_files(requests, requestPath, responses, responsePath):
    hpp_names = []
    for request in requests:
        fileName = convert_msg_to_hpp_include(request)[22:-1]
        hpp = convert_individual_hpp(request, requestPath, "request")
        hpp_names.append(fileName)
        with open(fileName, "w") as f:
            f.write(hpp)

    for response in responses:
        fileName = convert_msg_to_hpp_include(response)[22:-1]
        hpp = convert_individual_hpp(response, responsePath, "response")
        hpp_names.append(fileName)
        with open(fileName, "w") as f:
            f.write(hpp)
        
    return hpp_names

def convert_main_hpp_file(requests_msgs, response_msgs, hpp_names):
    hpp = "#pragma once\n\n#include <mutex>\n#include <string>\n#include <variant>\n#include <vector>\n\n#include <rj_common/time.hpp>\n#include <rj_convert/ros_convert.hpp>\n\n"
    hpp += "#include \"rj_msgs/msg/agent_request.hpp\"\n"
    hpp += "#include \"rj_msgs/msg/agent_response.hpp\"\n"
    hpp += "#include \"rj_msgs/msg/agent_response_variant.hpp\"\n"

    for name in hpp_names:
        hpp += "#include \"" + name + "\"\n"

    hpp += "\nnamespace strategy::communication {\n\n"
    hpp += "/**\n"
    hpp += "* @brief a conglomeration of the different request types.\n"
    hpp += "*/\n"
    hpp += "using AgentRequest = std::variant<"
    for request in requests_msgs:
        msgName = request[:-4]
        hpp += msgName + ", "

    hpp = hpp[:-2] + ">;\n\n"

    hpp += "/**\n"
    hpp += "* @brief a conglomeration of the different response types.\n"
    hpp += "*/\n"
    hpp += "using AgentResponseVariant = std::variant<"
    for response in response_msgs:
        msgName = response[:-4]
        hpp += msgName + ", "
    hpp = hpp[:-2] + ">;\n\n"

    hpp += "/**\n"
    hpp += "* @brief response message that is sent from the receiver of the request to the\n"
    hpp += "* sender of the request with an accompanying response.\n"
    hpp += "*\n"
    hpp += "* The agent response is the actual thing that gets sent from a receiver back\n"
    hpp += "* to the sender.\n"
    hpp += "*\n"
    hpp += "*/\n"
    hpp += "struct AgentResponse {\n"
    hpp += "\tAgentRequest associated_request;\n"
    hpp += "\tAgentResponseVariant response;\n"
    hpp += "};\n\n"

    hpp += "bool operator==(const AgentResponse& a, const AgentResponse& b);\n\n"

    hpp += "/**\n"
    hpp += "* @brief Wraps a communication request by giving the intended destination of the\n"
    hpp += "* communication.\n"
    hpp += "*\n"
    hpp += "* positions will create this and send it to their agent action client which will\n"
    hpp += "* send out the request according to their specifications.\n"
    hpp += "*\n"
    hpp += "*/\n"

    hpp += "struct PosAgentRequestWrapper {\n"
    hpp += "\tAgentRequest request;\n"
    hpp += "\tstd::vector<u_int8_t> target_agents;\n"
    hpp += "\tbool broadcast;\n"
    hpp += "\tbool urgent;\n"
    hpp += "};\n\n"

    hpp += "/**\n"
    hpp += "* @brief Wraps a communication response to ensure symmetry for agent-to-agent\n"
    hpp += "* communication.\n"
    hpp += "*\n"
    hpp += "* this wrapper is placed on agent responses to promote symmetry across the request\n"
    hpp += "* response system to make understanding easier.  All this struct does is make explicit\n"
    hpp += "* that this response is going from the position to the agent.\n"
    hpp += "*\n"
    hpp += "*/\n"
    hpp += "struct PosAgentResponseWrapper {\n"
    hpp += "\tAgentResponseVariant response;\n"
    hpp += "};\n\n"

    hpp += "/**\n"
    hpp += "* @brief Wraps a communication request to ensure symmetry for agent-to-agent\n"
    hpp += "* communication.\n"
    hpp += "*\n"
    hpp += "* Like the PosAgentResponseWrapper, this struct does nothing other than make the request\n"
    hpp += "* response system more symmetrical and (hopefully) more easy to understand.  All this struct\n"
    hpp += "* does is make it explicit that this request is being passed from the agent to the agent to\n"
    hpp += "* the position.\n"
    hpp += "*\n"
    hpp += "*/\n"
    hpp += "struct AgentPosRequestWrapper {\n"
    hpp += "\tAgentRequest request;\n"
    hpp += "};\n\n"

    hpp += "/**\n"
    hpp += "* @brief Wraps a communication response by giving the robot the communication is from.\n"
    hpp += "*\n"
    hpp += "* the AgentPosResponseWrapper is the actual thing being passed from the agent to the position\n"
    hpp += "* once either the timeout period was reached or enough responses were received.  Ideally, the\n"
    hpp += "* contents of this wrapper should contain all of the non-message specific fields that a position\n"
    hpp += "* will need to handle a response.\n"
    hpp += "*\n"
    hpp += "*/\n"
    hpp += "struct AgentPosResponseWrapper {\n"
    hpp += "\tAgentRequest associated_request;\n"
    hpp += "\tstd::vector<u_int8_t> to_robot_ids;\n"
    hpp += "\tstd::vector<u_int8_t> received_robot_ids;\n"
    hpp += "\tbool broadcast;\n"
    hpp += "\tbool urgent;\n"
    hpp += "\tRJ::Time created;\n"
    hpp += "\tstd::vector<AgentResponseVariant> responses;\n"
    hpp += "};\n\n"
    hpp += "}\n\n"

    hpp += "namespace rj_convert {\n\n"
    hpp += "template <>\n"
    hpp += "struct RosConverter<strategy::communication::AgentRequest, rj_msgs::msg::AgentRequest> {\n"
    hpp += "\tstatic rj_msgs::msg::AgentRequest to_ros(const strategy::communication::AgentRequest& from) {\n"
    hpp += "\t\trj_msgs::msg::AgentRequest result;\n"
    hpp += "\t\t"
    for request in requests_msgs:
        msgName = convert_msg_to_hpp_include(request)[22:-5]
        hpp += "if (const auto* " + msgName + " = std::get_if<strategy::communication::" + request[:-4] + ">(&from)) {\n"
        hpp += "\t\t\tresult." + msgName + ".emplace_back(convert_to_ros(*" + msgName + "));\n"
        hpp += "\t\t} else "
    hpp += "{\n"
    hpp += "\t\t\tthrow std::runtime_error(\"Invalid variant of AgentRequest\");\n"
    hpp += "\t\t}\n"
    hpp += "\t\treturn result;\n"
    hpp += "\t}\n\n"

    hpp += "\tstatic strategy::communication::AgentRequest from_ros(const rj_msgs::msg::AgentRequest& from) {\n"
    hpp += "\t\tstrategy::communication::AgentRequest result;\n\t\t"
    for request in requests_msgs:
        msgName = convert_msg_to_hpp_include(request)[22:-5]
        hpp += "if (from." + msgName + ".empty()) {\n"
        hpp += "\t\t\tresult = convert_from_ros(from." + msgName + ".front());\n"
        hpp += "\t\t} else "
    hpp += "{\n"
    hpp += "\t\t\tthrow std::runtime_error(\"Invalid variant of AgentRequest\");\n"
    hpp += "\t\t}\n"
    hpp += "\t\treturn result;\n"
    hpp += "\t}\n\n"
    hpp += "};\n\n"
    hpp += "ASSOCIATE_CPP_ROS(strategy::communication::AgentRequest, rj_msgs::msg::AgentRequest);\n\n"

    hpp += "template <>\n"
    hpp += "struct RosConverter<strategy::communication::AgentResponse, rj_msgs::msg::AgentResponse> {\n"
    hpp += "\tstatic rj_msgs::msg::AgentResponse to_ros(const strategy::communication::AgentResponse& from) {\n"
    hpp += "\t\trj_msgs::msg::AgentResponse result;\n"
    hpp += "\t\tresult.associated_request = convert_to_ros(from.associated_request);\n\t\t"
    for response in response_msgs:
        msgName = convert_msg_to_hpp_include(response)[22:-5]
        hpp += "if (const auto* " + msgName + " = std::get_if<strategy::communication::" + response[:-4] + ">(&(from.response))) {\n"
        hpp += "\t\t\tresult." + msgName + ".emplace_back(convert_to_ros(*" + msgName + "));\n"
        hpp += "\t\t} else "
    hpp += "{\n"
    hpp += "\t\t\tthrow std::runtime_error(\"Invalid variant of AgentResponse\");\n"
    hpp += "\t\t}\n"
    hpp += "\t\treturn result;\n"
    hpp += "\t}\n\n"

    hpp += "\tstatic strategy::communication::AgentResponse from_ros(const rj_msgs::msg::AgentResponse& from) {\n"
    hpp += "\t\tstrategy::communication::AgentResponse result;\n"
    hpp += "\t\tresult.associated_request = convert_from_ros(from.associated_request);\n\t\t"
    for response in response_msgs:
        msgName = convert_msg_to_hpp_include(response)[22:-5]
        hpp += "if (from." + msgName + ".empty()) {\n"
        hpp += "\t\t\tresult = convert_from_ros(from." + msgName + ".front());\n"
        hpp += "\t\t} else "
    hpp += "{\n"
    hpp += "\t\t\tthrow std::runtime_error(\"Invalid variant of AgentResponse\");\n"
    hpp += "\t\t}\n"
    hpp += "\t\treturn result;\n"
    hpp += "\t}\n\n"
    hpp += "};\n\n"

    hpp += "ASSOCIATE_CPP_ROS(strategy::communication::AgentResponse, rj_msgs::msg::AgentResponse);\n\n"
    hpp += "}"
    return hpp

def create_main_hpp_file(requests_msgs, response_msgs, hpp_names):
    hpp = convert_main_hpp_file(requests_msgs, response_msgs, hpp_names)
    with open("communication.hpp", "w") as f:
        f.write(hpp)

if __name__ == "__main__":
    path_request = "../../../../../../rj_msgs/request"
    path_response = "../../../../../../rj_msgs/response"

    requests_msgs = os.listdir(path_request)
    response_msgs = os.listdir(path_response)

    hpp_names = create_hpp_files(requests_msgs, path_request, response_msgs, path_response)
    create_cpp_file(requests_msgs, response_msgs, hpp_names)
    create_main_hpp_file(requests_msgs, response_msgs, hpp_names)

        