#include <RJLog.hpp>

#include <vector>
#include <string>
#include <iostream>
#include <fstream>

using namespace std;

RJLog::RJLog() {};

RJLog::RJLog(string filename) {
    ifstream in(filename.c_str(), ios_base::in | ios_base::binary);

    if (!in.is_open()) {
        cerr << "Error opening log file \"" << filename << "\"!" << endl;
    }
}

/**
 * Read frames and parse protobuf from specified file
 * @param filename Filename of a RoboJackets protobuf log file
 * @return true If reading and parsing was flawless
 * @return false If there was any errors or warnings while parsing
 *
bool readFrames(const char* filename) {
    QFile file(filename);

    if (!file.open(QFile::ReadOnly)) {
        fprintf(stderr, "Can't open %s: %s\n", filename,
                (const char*)file.errorString().toLatin1());
        return false;
    }

    while (!file.atEnd()) {
        uint32_t size = 0;
        if (file.read((char*)&size, sizeof(size)) != sizeof(size)) {
            // Broken length
            printf("Broken length\n");
            return false;
        }

        string str(size, 0);
        if (file.read(&str[0], size) != size) {
            // Broken packet at end of file
            printf("Broken packet\n");
            return false;
        }

        shared_ptr<LogFrame> frame = std::make_shared<LogFrame>();
        frames.push_back(frame);
        // Parse partial so we can recover from corrupt data
        if (!frame->ParsePartialFromString(str)) {
            printf("Failed: %s\n", frame->InitializationErrorString().c_str());
            return false;
        }
    }

    return true;
}

*/