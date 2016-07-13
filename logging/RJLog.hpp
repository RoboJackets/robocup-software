#pragma once

#include <protobuf/LogFrame.pb.h>
#include <memory>
#include <vector>

using namespace std;

class RJLog {
public:
    RJLog();
    RJLog(string filename);
    vector<shared_ptr<Packet::LogFrame>> getFrames();

private:
    vector<shared_ptr<Packet::LogFrame>> frames;
};
