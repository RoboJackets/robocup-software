#pragma once

#include <RJLog.hpp>

class SSLLog {
public:
    SSLLog(string filename);
    SSLLog(RJLog rjLog);
    RJLog toRJ();

private:

vector<shared_ptr<SSL_WrapperPacket>> vision2014;
vector<shared_ptr<SSL_Referee>> referee;

struct FileHeader {
    char name[12]; // SSL_LOG_FILE
    int32_t version; // Default file format is version 1
};

const char* DEFAULT_FILE_HEADER_NAME = "SSL_LOG_FILE";
const int32_t DEFAULT_FILE_VERSION = 1;

struct DataHeader {
    int64_t timestamp; // Timestamp in ns
    int32_t messageType; // Message type
    int32_t messageSize; // Size of protobuf message in bytes
};

enum MessageType {
    MESSAGE_BLANK = 0,
    MESSAGE_UNKNOWN = 1,
    MESSAGE_SSL_VISION_2010 = 2,
    MESSAGE_SSL_REFBOX_2013 = 3,
    MESSAGE_SSL_VISION_2014 = 4
};
};