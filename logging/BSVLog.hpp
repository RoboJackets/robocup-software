#pragma once

#include <RJLog.hpp>
#include <memory>

using namespace std;

const int MATCH_ID_LENGTH = 32;
const char BAR = '|';

class BSVLog {
public:
    BSVLog(RJLog rjLog);
    void write(const char* robotFilename, const char* frameFilename);

private:
    char matchID[MATCH_ID_LENGTH + 1];  // +1 adds room for null terminator
    struct robotLine {
        char* matchID;
        long long timestamp;
        string team;
        int shellID;
        float xPosition;
        float yPosition;
        float angle;
    };
    struct frameLine {
        char* matchID;
        long long timestamp;
        float ballXPosition;
        float ballYPosition;
        int yellowScore;
        int blueScore;
        int yellowGoalie;
        int blueGoalie;
        int stage;
        int command;
    };
    void generateMatchID(char* id, int length);
    vector<shared_ptr<robotLine>> robots;
    vector<shared_ptr<frameLine>> frames;
};