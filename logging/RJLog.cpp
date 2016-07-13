#include <RJLog.hpp>

#include <protobuf/LogFrame.pb.h>

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <memory>

using namespace std;
using namespace Packet;

RJLog::RJLog(){};

RJLog::RJLog(string filename) {
    ifstream in(filename.c_str(), ios_base::in | ios_base::binary);

    if (!in.is_open()) {
        cerr << "Error opening log file \"" << filename << "\"!" << endl;
    }

    while (!in.eof()) {
        uint32_t size = 0;
        in.read((char*)&size, sizeof(size));

        string str(size, 0);
        in.read(&str[0], size);

        shared_ptr<LogFrame> frame = make_shared<LogFrame>();
        frames.push_back(frame);
        // Parse partial so we can recover from corrupt data
        if (!frame->ParsePartialFromString(str)) {
            printf("Failed: %s\n", frame->InitializationErrorString().c_str());
        }
    }
}

vector<shared_ptr<LogFrame>> RJLog::getFrames() { return frames; }