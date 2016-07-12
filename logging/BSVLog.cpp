#include <BSVLog.hpp>
#include <RJLog.hpp>

#include <unistd.h>

using namespace std;

BSVLog::BSVLog(RJLog rjLog) {}

void BSVLog::write(const char* robotFilename, const char* frameFilename) {
    /*
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

            using namespace NewRefereeModuleEnums;
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
             */
}

/**
 * Generates random alphanumeric strings using PID and time as the random seed.
 * A Null terminator is added after the ID
 * @param id A pointer to a character array that can hold at least length + 1
 * chars
 * @param length Length of the ID to be generated.
 */
void BSVLog::generateMatchID(char* id, int length) {
    string chars(
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890");

    // Avoid using time based srand init
    srand(getpid() * time(NULL));

    for (int i = 0; i < length; ++i) {
        id[i] = chars[rand() % chars.length()];
    }

    id[length] = '\0';
}