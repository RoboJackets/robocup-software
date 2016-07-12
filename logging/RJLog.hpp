#pragma once

#include <protobuf/LogFrame.pb.h>
#include <memory>
#include <vector>

using namespace std;

class RJLog {
public:
    RJLog();
    RJLog(string filename);

private:
    vector<shared_ptr<Packet::LogFrame>> frames;
};
