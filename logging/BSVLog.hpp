#pragma once

#include <RJLog.hpp>
#include <NewRefereeModule.hpp>

#include <memory>

using namespace std;
using namespace NewRefereeModuleEnums;

const int MATCH_ID_LENGTH = 32;
const char BAR = '|';
const string chars(
    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890");

struct RobotLine {
    long long timestamp;
    string team;
    int shellID;
    float xPosition;
    float yPosition;
    float angle;
};
struct FrameLine {
    long long timestamp;
    float ballXPosition;
    float ballYPosition;
    int yellowScore;
    int blueScore;
    int yellowGoalie;
    int blueGoalie;
    Stage stage;
    Command command;
};

class BSVLog {
public:
    BSVLog(RJLog rjLog);
    void write(string robotFilename, string frameFilename);

private:
    char matchID[MATCH_ID_LENGTH + 1];  // +1 adds room for null terminator
    void generateMatchID();
    vector<shared_ptr<RobotLine>> robots;
    vector<shared_ptr<FrameLine>> frames;
};