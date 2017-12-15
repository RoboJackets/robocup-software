#pragma once

#include "firmware-common/common2015/utils/rtp.hpp"

namespace DebugCommunication {
    const std::map<DebugResponse,std::string> DEBUGRESPONSE_TO_STRING = {
            {DebugResponse::PIDError0, "PIDError0"},
            {DebugResponse::PIDError1, "PIDError1"},
            {DebugResponse::PIDError2, "PIDError2"},
            {DebugResponse::PIDError3, "PIDError3"},
            {DebugResponse::MotorDuty0, "MotorDuty0"},
            {DebugResponse::MotorDuty1, "MotorDuty1"},
            {DebugResponse::MotorDuty2, "MotorDuty2"},
            {DebugResponse::MotorDuty3, "MotorDuty3"},
            {DebugResponse::WheelVel0, "WheelVel0"},
            {DebugResponse::WheelVel1, "WheelVel1"},
            {DebugResponse::WheelVel2, "WheelVel2"},
            {DebugResponse::WheelVel3, "WheelVel3"},
            {DebugResponse::StallCounter0, "StallCounter0"},
            {DebugResponse::StallCounter1, "StallCounter1"},
            {DebugResponse::StallCounter2, "StallCounter2"},
            {DebugResponse::StallCounter3, "StallCounter3"}, 
            {DebugResponse::TargetWheelVel0, "TargetWheelVel0"},
            {DebugResponse::TargetWheelVel1, "TargetWheelVel1"},
            {DebugResponse::TargetWheelVel2, "TargetWheelVel2"},
            {DebugResponse::TargetWheelVel3, "TargetWheelVel3"}
    };

    const std::map<std::string ,DebugResponse> STRING_TO_DEBUGRESPONSE = [](){
        std::map<std::string ,DebugResponse> m{};
        for (const auto& pair: DEBUGRESPONSE_TO_STRING) {
            m[pair.second] = pair.first;
        }
        debugThrowIf("Enums missing from RESPONSE_TO_STRING", DEBUGRESPONSE_TO_STRING.size() != RESPONSE_INFO.size());
        return m;
    }();

    const std::map<ConfigCommunication,std::string> CONFIG_TO_STRING = {
        {ConfigCommunication::PID_P, "PID_P"},
        {ConfigCommunication::PID_I, "PID_I"},
        {ConfigCommunication::PID_D, "PID_D"}
    };

    const std::map<std::string ,ConfigCommunication> STRING_TO_CONFIG = [](){
        std::map<std::string ,ConfigCommunication> m{};
        for (const auto& pair: CONFIG_TO_STRING) {
            m[pair.second] = pair.first;
        }
        debugThrowIf("Enums missing from STRING_TO_CONFIG", STRING_TO_CONFIG.size() != CONFIG_INFO.size());
        return m;
    }();
}