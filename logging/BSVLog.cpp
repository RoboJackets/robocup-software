#include <BSVLog.hpp>
#include <RJLog.hpp>
#include <NewRefereeModule.hpp>

#include <unistd.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace Packet;

BSVLog::BSVLog(RJLog rjLog) {
    generateMatchID();

    vector<shared_ptr<LogFrame>> rjFrames = rjLog.getFrames();

    for (int i = 0; i < rjFrames.size(); i++) {
        LogFrame* currentFrame = rjFrames[i].get();

        shared_ptr<FrameLine> frame = make_shared<FrameLine>();
        frame->timestamp = currentFrame->timestamp();

        bool blue_team = currentFrame->blue_team();
        string selfTeam = blue_team ? "blue" : "yellow";
        string oppTeam = !blue_team ? "blue" : "yellow";

        // loop over repeated self value
        for (int i = 0; i < currentFrame->self_size(); i++) {
            shared_ptr<RobotLine> robot = make_shared<RobotLine>();
            robot->timestamp = currentFrame->timestamp();
            robot->team = selfTeam;

            LogFrame_Robot currentRobot = currentFrame->self(i);

            robot->angle = currentRobot.angle();
            robot->shellID = currentRobot.shell();

            Point position = currentRobot.pos();
            robot->xPosition = position.x();
            robot->yPosition = position.y();

            robots.push_back(robot);
        }
        for (int i = 0; i < currentFrame->opp_size(); i++) {
            shared_ptr<RobotLine> robot = make_shared<RobotLine>();
            robot->timestamp = currentFrame->timestamp();
            robot->team = oppTeam;

            LogFrame_Robot currentRobot = currentFrame->opp(i);

            robot->angle = currentRobot.angle();
            robot->shellID = currentRobot.shell();

            Point position = currentRobot.pos();
            robot->xPosition = position.x();
            robot->yPosition = position.y();

            robots.push_back(robot);
        }

        // Get information on the ball
        LogFrame_Ball ball = currentFrame->ball();
        Point ballPosition = ball.pos();

        frame->ballXPosition = ballPosition.x();
        frame->ballYPosition = ballPosition.y();

        if (currentFrame->raw_refbox_size() > 0) {
            SSL_Referee referee = currentFrame->raw_refbox(0);

            using namespace NewRefereeModuleEnums;
            frame->stage = (Stage)referee.stage();
            frame->command = (Command)referee.command();

            frame->blueScore = referee.blue().score();
            frame->yellowScore = referee.yellow().score();

            frame->blueGoalie = referee.blue().goalie();
            frame->yellowGoalie = referee.yellow().goalie();
        }

        frames.push_back(frame);

    }
    cout << "BSV Generated " << robots.size() << " Robot lines and "
         << frames.size() << " Frame lines" << endl;
    
}

void BSVLog::write(string robotFilename, string frameFilename) {
    ofstream outRobot(robotFilename.c_str(), ios_base::out);
    ofstream outFrame(frameFilename.c_str(), ios_base::out);

    outRobot << "match_id" << BAR << "timestamp" << BAR << "team" << BAR
             << "shell_id" << BAR << "x_position" << BAR << "y_position" << BAR
             << "angle" << endl; 

    for (int i = 0; i < robots.size(); ++i) {
        outRobot << matchID << BAR << robots[i]->timestamp << BAR
                 << robots[i]->team << BAR << robots[i]->shellID << BAR                 << robots[i]->xPosition << BAR << robots[i]->yPosition << BAR
                 << robots[i]->angle << endl;
    }

    outFrame << "match_id" << BAR << "timestamp" << BAR << "ball_x_position"
             << BAR << "ball_y_position" << BAR << "stage" << BAR << "command"
             << BAR << "yellow_score" << BAR << "blue_score" << BAR
             << "yellow_goalie" << BAR << "blue_goalie" << endl;

    for (int i = 0; i < frames.size(); ++i) {
        outFrame << matchID << BAR << frames[i]->timestamp << BAR
                 << frames[i]->ballXPosition << BAR << frames[i]->ballYPosition
                 << BAR << frames[i]->stage << BAR << frames[i]->command << BAR
                 << frames[i]->yellowScore << BAR << frames[i]->blueScore << BAR
                 << frames[i]->yellowGoalie << BAR << frames[i]->blueGoalie
                 << endl;
    }
}

/**
 * Generates random alphanumeric strings using PID and time as the random
 * seed.
 * A Null terminator is added after the ID
 */
void BSVLog::generateMatchID() {
    // Avoid using time based srand init
    srand(getpid() * time(NULL));

    for (int i = 0; i < MATCH_ID_LENGTH + 1; ++i) {
        matchID[i] = chars[rand() % chars.length()];
    }

    matchID[MATCH_ID_LENGTH + 1] = '\0';
}