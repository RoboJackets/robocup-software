#include <referee_enums.h>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <protobuf/LogFrame.pb.h>
#include <protobuf/referee.pb.h>

#include <QFile>
#include <QTextStream>

#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <ctime>

using namespace Packet;
using namespace std;

const char BAR = '|';
const int MATCH_ID_LENGTH = 32;

vector<std::shared_ptr<Packet::LogFrame> > frames;

/**
 * Defines usage information for launching the Log-Viewer application
 * @param prog The name of the program
 */
void usage(const char* prog) {
    fprintf(stderr, "Usage: %s <filename.log> <output base filename>\n", prog);
    exit(1);
}

/**
 * Generates random alphanumeric strings using PID and time as the random seed.
 * A Null terminator is added after the ID
 * @param id A pointer to a character array that can hold at least length + 1
 * chars
 * @param length Length of the ID to be generated.
 */
void generateMatchID(char* id, int length) {
    string chars(
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890");

    // Avoid using time based srand init
    srand(getpid() * time(NULL));

    for (int i = 0; i < length; ++i) {
        id[i] = chars[rand() % chars.length()];
    }

    id[length] = '\0';
}

/**
 * Read frames and parse protobuf from specified file
 * @param filename Filename of a RoboJackets protobuf log file
 * @return true If reading and parsing was flawless
 * @return false If there was any errors or warnings while parsing
 */
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

int main(int argc, char* argv[]) {
    if (argc != 3) {
        usage(argv[0]);
    }

    const char* logFilename = argv[1];

    fprintf(stderr, "Exporting BSV file to %s\n", argv[2]);

    string bsvBase(argv[2]);

    string robotFilename = bsvBase + "-robot.bsv";
    QFile fileRobot(robotFilename.c_str());
    if (!fileRobot.open(QIODevice::WriteOnly | QIODevice::Text)) return false;
    QTextStream outRobot(&fileRobot);

    string frameFilename = bsvBase + "-frame.bsv";
    QFile fileFrame(frameFilename.c_str());
    if (!fileFrame.open(QIODevice::WriteOnly | QIODevice::Text)) return false;
    QTextStream outFrame(&fileFrame);

    // Read in data from log file
    if (readFrames(logFilename) == false) {
        exit(1);
    }

    char matchID[MATCH_ID_LENGTH + 1];

    generateMatchID(matchID, MATCH_ID_LENGTH);

    fprintf(stderr, "matchID: %s\n", matchID);

    // Print Headers
    outRobot << "match_id" << BAR << "timestamp" << BAR << "team" << BAR
             << "shell_id" << BAR << "x_position" << BAR << "y_position" << BAR
             << "angle" << endl;
    outFrame << "match_id" << BAR << "timestamp" << BAR << "ball_x_position"
             << BAR << "ball_y_position" << BAR << "stage" << BAR << "command"
             << BAR << "yellow_score" << BAR << "blue_score" << BAR
             << "yellow_goalie" << BAR << "blue_goalie" << endl;

    for (int i = 0; i < frames.size(); i++) {
        LogFrame* currentFrame = frames[i].get();

        const long long timestamp = currentFrame->timestamp();

        const bool blue_team = currentFrame->blue_team();
        const char* selfTeam = blue_team ? "blue" : "yellow";
        const char* oppTeam = !blue_team ? "blue" : "yellow";

        // loop over repeated self value
        for (int i = 0; i < currentFrame->self_size(); i++) {
            LogFrame_Robot currentRobot = currentFrame->self(i);

            const float angle = currentRobot.angle();
            const int shell = currentRobot.shell();

            Point position = currentRobot.pos();
            const float xPos = position.x();
            const float yPos = position.y();

            // Construct a row of data and print to file
            outRobot << matchID << BAR << timestamp << BAR << selfTeam << BAR
                     << shell << BAR << xPos << BAR << yPos << BAR << angle
                     << endl;
        }

        for (int i = 0; i < currentFrame->opp_size(); i++) {
            LogFrame_Robot currentRobot = currentFrame->opp(i);

            const float angle = currentRobot.angle();
            const int shell = currentRobot.shell();

            Point position = currentRobot.pos();
            const float xPos = position.x();
            const float yPos = position.y();

            // Construct a row of data and print to file
            outRobot << matchID << BAR << timestamp << BAR << oppTeam << BAR
                     << shell << BAR << xPos << BAR << yPos << BAR << angle
                     << endl;
        }

        // Get information on the ball
        LogFrame_Ball ball = currentFrame->ball();
        Point ballPosition = ball.pos();

        const float xBallPos = ballPosition.x();
        const float yBallPos = ballPosition.y();

        string stage;
        string command;

        int blueScore = 0;
        int yellowScore = 0;

        int blueGoalie = 0;
        int yellowGoalie = 0;

        if (currentFrame->raw_refbox_size() > 0) {
            SSL_Referee referee = currentFrame->raw_refbox(0);

            using namespace RefereeModuleEnums;
            stage = stringFromStage((Stage)referee.stage());
            command = stringFromCommand((Command)referee.command());

            blueScore = referee.blue().score();
            yellowScore = referee.yellow().score();

            blueGoalie = referee.blue().goalie();
            yellowGoalie = referee.yellow().goalie();
        }

        const char* c_stage = stage.c_str();
        const char* c_command = command.c_str();

        outFrame << matchID << BAR << timestamp << BAR << xBallPos << BAR
                 << yBallPos << BAR << c_stage << BAR << c_command << BAR
                 << yellowScore << BAR << blueScore << BAR << yellowGoalie
                 << BAR << blueGoalie << endl;
    }
};
