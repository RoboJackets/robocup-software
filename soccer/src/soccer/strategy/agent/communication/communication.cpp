#include "communication.hpp"

namespace strategy::communication {

std::mutex request_uid_mutex;
u_int32_t request_uid = 0;

std::mutex response_uid_mutex;
u_int32_t response_uid = 0;

bool operator==(const LeaveWallRequest& a, const LeaveWallRequest& b) {
    return a.request_uid == b.request_uid;
}

bool operator==(const PassRequest& a, const PassRequest& b) {
    return a.request_uid == b.request_uid;
}

bool operator==(const TestRequest& a, const TestRequest& b) {
    return a.request_uid == b.request_uid;
}

bool operator==(const BallInTransitRequest& a, const BallInTransitRequest& b) {
    return a.request_uid == b.request_uid;
}

bool operator==(const KickerRequest& a, const KickerRequest& b) {
    return a.request_uid == b.request_uid;
}

bool operator==(const JoinWallRequest& a, const JoinWallRequest& b) {
    return a.request_uid == b.request_uid;
}

bool operator==(const ScorerRequest& a, const ScorerRequest& b) {
    return a.request_uid == b.request_uid;
}

bool operator==(const IncomingBallRequest& a, const IncomingBallRequest& b) {
    return a.request_uid == b.request_uid;
}

bool operator==(const ResetScorerRequest& a, const ResetScorerRequest& b) {
    return a.request_uid == b.request_uid;
}

bool operator==(const PositionRequest& a, const PositionRequest& b) {
    return a.request_uid == b.request_uid;
}

bool operator==(const LeaveWallResponse& a, const LeaveWallResponse& b) {
    return a.response_uid == b.response_uid;
}

bool operator==(const ScorerResponse& a, const ScorerResponse& b) {
    return a.response_uid == b.response_uid;
}

bool operator==(const JoinWallResponse& a, const JoinWallResponse& b) {
    return a.response_uid == b.response_uid;
}

bool operator==(const Acknowledge& a, const Acknowledge& b) {
    return a.response_uid == b.response_uid;
}

bool operator==(const PositionResponse& a, const PositionResponse& b) {
    return a.response_uid == b.response_uid;
}

bool operator==(const PassResponse& a, const PassResponse& b) {
    return a.response_uid == b.response_uid;
}

bool operator==(const TestResponse& a, const TestResponse& b) {
    return a.response_uid == b.response_uid;
}

bool operator==(const AgentResponse& a, const AgentResponse& b) {
    return (a.associated_request == b.associated_request) && (a.response == b.response);
}

void generate_uid(LeaveWallRequest& request) {
    request_uid_mutex.lock();
    request.request_uid = request_uid;
    request_uid++;
    request_uid_mutex.unlock();
}

void generate_uid(PassRequest& request) {
    request_uid_mutex.lock();
    request.request_uid = request_uid;
    request_uid++;
    request_uid_mutex.unlock();
}

void generate_uid(TestRequest& request) {
    request_uid_mutex.lock();
    request.request_uid = request_uid;
    request_uid++;
    request_uid_mutex.unlock();
}

void generate_uid(BallInTransitRequest& request) {
    request_uid_mutex.lock();
    request.request_uid = request_uid;
    request_uid++;
    request_uid_mutex.unlock();
}

void generate_uid(KickerRequest& request) {
    request_uid_mutex.lock();
    request.request_uid = request_uid;
    request_uid++;
    request_uid_mutex.unlock();
}

void generate_uid(JoinWallRequest& request) {
    request_uid_mutex.lock();
    request.request_uid = request_uid;
    request_uid++;
    request_uid_mutex.unlock();
}

void generate_uid(ScorerRequest& request) {
    request_uid_mutex.lock();
    request.request_uid = request_uid;
    request_uid++;
    request_uid_mutex.unlock();
}

void generate_uid(IncomingBallRequest& request) {
    request_uid_mutex.lock();
    request.request_uid = request_uid;
    request_uid++;
    request_uid_mutex.unlock();
}

void generate_uid(ResetScorerRequest& request) {
    request_uid_mutex.lock();
    request.request_uid = request_uid;
    request_uid++;
    request_uid_mutex.unlock();
}

void generate_uid(PositionRequest& request) {
    request_uid_mutex.lock();
    request.request_uid = request_uid;
    request_uid++;
    request_uid_mutex.unlock();
}

void generate_uid(LeaveWallResponse& response) {
    response_uid_mutex.lock();
    response.response_uid = response_uid;
    response_uid++;
    response_uid_mutex.unlock();
}

void generate_uid(ScorerResponse& response) {
    response_uid_mutex.lock();
    response.response_uid = response_uid;
    response_uid++;
    response_uid_mutex.unlock();
}

void generate_uid(JoinWallResponse& response) {
    response_uid_mutex.lock();
    response.response_uid = response_uid;
    response_uid++;
    response_uid_mutex.unlock();
}

void generate_uid(Acknowledge& response) {
    response_uid_mutex.lock();
    response.response_uid = response_uid;
    response_uid++;
    response_uid_mutex.unlock();
}

void generate_uid(PositionResponse& response) {
    response_uid_mutex.lock();
    response.response_uid = response_uid;
    response_uid++;
    response_uid_mutex.unlock();
}

void generate_uid(PassResponse& response) {
    response_uid_mutex.lock();
    response.response_uid = response_uid;
    response_uid++;
    response_uid_mutex.unlock();
}

void generate_uid(TestResponse& response) {
    response_uid_mutex.lock();
    response.response_uid = response_uid;
    response_uid++;
    response_uid_mutex.unlock();
}

}  // namespace strategy::communication