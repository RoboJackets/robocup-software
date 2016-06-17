#include <LogViewer.hpp>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <protobuf/LogFrame.pb.h>
#include <protobuf/referee.pb.h>

#include <QApplication>
#include <QFile>
#include <QTextStream>

#include <algorithm>
#include <fcntl.h>

using namespace std;
using namespace boost;
using namespace Packet;
using namespace google::protobuf::io;
using namespace google::protobuf;

void usage(const char* prog) {
    fprintf(stderr, "Usage: %s [options] <filename.log>\n", prog);
    fprintf(stderr,
            "\t-bsv <Filename.bsv>: Exports the log file to a BSV of the same "
            "name and Prevents launch of GUI\n");
    fprintf(stderr, "\t-help:       Displays this help text\n");
    exit(1);
}

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    bool bsv = false;
    char* bsvFilename;
    char* logFilename;

    // initial check for number of args
    if (argc < 2 || argc > 4) {
        usage(argv[0]);
    }

    // Process args
    for (int i = 1; i < argc; ++i) {
        const char* var = argv[i];

        if (strcmp(var, "-bsv") == 0) {
            bsv = true;
            ++i;
            bsvFilename = argv[i];
        } else if (strcmp(var, "-help") == 0) {
            usage(argv[0]);
        } else {
            logFilename = argv[i];
        }
    }

    LogViewer win;

    if (bsv) {
        // Export BSV. Runs log_viewer in headless mode
        if (win.exportBSV(logFilename, bsvFilename) == false) {
            return 0;
        }
    } else {
        // Views the log in the log_view GUI
        win.launchGUI(logFilename);
        return app.exec();
    }

    return 1;
}

LogViewer::LogViewer(QWidget* parent) : QMainWindow(parent) {
    ui.setupUi(this);

    _history.resize(2 * 60);
    ui.fieldView->history(&_history);

    _frameNumberItem = new QTreeWidgetItem(ui.tree);
    _frameNumberItem->setText(ProtobufTree::Column_Field, "Frame");
    _frameNumberItem->setData(ProtobufTree::Column_Tag, Qt::DisplayRole, -2);

    _elapsedTimeItem = new QTreeWidgetItem(ui.tree);
    _elapsedTimeItem->setText(ProtobufTree::Column_Field, "Elapsed Time");
    _elapsedTimeItem->setData(ProtobufTree::Column_Tag, Qt::DisplayRole, -1);

    ui.splitter->setStretchFactor(0, 98);
    ui.splitter->setStretchFactor(1, 10);

    QActionGroup* rotateGroup = new QActionGroup(this);
    rotateGroup->addAction(ui.action0);
    rotateGroup->addAction(ui.action90);
    rotateGroup->addAction(ui.action180);
    rotateGroup->addAction(ui.action270);

    connect(&_updateTimer, SIGNAL(timeout()), SLOT(updateViews()));
    _updateTimer.start(30);
}

// Parses log File & exports a BSV file containing the data of the log file
bool LogViewer::exportBSV(char* logFilename, char* bsvFilename) {
    fprintf(stderr, "Exporting BSV file to %s\n", bsvFilename);

    string bsvBase(bsvFilename);

    string robotFilename = bsvBase + "-robot.bsv";
    QFile fileRobot(robotFilename.c_str());
    if (!fileRobot.open(QIODevice::WriteOnly | QIODevice::Text)) return false;
    QTextStream outRobot(&fileRobot);

    string frameFilename = bsvBase + "-frame.bsv";
    QFile fileFrame(frameFilename.c_str());
    if (!fileFrame.open(QIODevice::WriteOnly | QIODevice::Text)) return false;
    QTextStream outFrame(&fileFrame);

    // Read in data from log file
    this->readFrames(logFilename);

    char matchID[MATCH_ID_LENGTH + 1];

    generateMatchID(matchID);

    fprintf(stderr, "matchID: %s\n", matchID);

    // Print Headers
    outRobot << "match_id" << BAR << "timestamp" << BAR << "team" << BAR
             << "shell_id" << BAR << "x_position" << BAR << "y_position" << BAR
             << "angle" << endl;
    outFrame << "match_id" << BAR << "timestamp" << BAR << "stage" << BAR
             << "command" << BAR << "ball_x_position" << BAR
             << "ball_y_posiiton" << endl;

    for (int i = 0; i < frames.size(); i++) {
        LogFrame* currentFrame = frames[i].get();

        const uint64 timestamp = currentFrame->timestamp();

        const bool blue_team = currentFrame->blue_team();
        const char* selfTeam = blue_team ? "blue" : "yellow";
        const char* oppTeam = !blue_team ? "blue" : "yellow";

        // loop over repeated self value
        for (int i = 0; i < currentFrame->self_size(); i++) {
            LogFrame_Robot currentRobot = currentFrame->self(i);

            const float angle = currentRobot.angle();
            const int32 shell = currentRobot.shell();

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
            const int32 shell = currentRobot.shell();

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

        // Get Referee data
        SSL_Referee_Stage stage;
        SSL_Referee_Command command;

        if (currentFrame->raw_refbox_size() > 0) {
            SSL_Referee referee = currentFrame->raw_refbox(0);

            stage = referee.stage();
            command = referee.command();
        } else {
            fprintf(stderr, "No Ref Data\n");
        }

        outFrame << matchID << BAR << timestamp << BAR << stage << BAR
                 << command << BAR << xBallPos << BAR << yBallPos << endl;
    }

    return true;
}

// Prepare program for GUI launch
bool LogViewer::launchGUI(const char* logFilename) {
    frames.clear();
    ui.timeSlider->setMaximum(0);

    bool read = this->readFrames(logFilename);

    ui.timeSlider->setMaximum(frames.size());

    this->showMaximized();
    return read;
}

// Read frames of logged data from specified file
bool LogViewer::readFrames(const char* filename) {
    QFile file(filename);

    if (!file.open(QFile::ReadOnly)) {
        fprintf(stderr, "Can't open %s: %s\n", filename,
                (const char*)file.errorString().toLatin1());
        return false;
    }

    int n = 0;
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

        std::shared_ptr<LogFrame> frame = std::make_shared<LogFrame>();
        frames.push_back(frame);
        // Parse partial so we can recover from corrupt data
        if (!frame->ParsePartialFromString(str)) {
            printf("Failed: %s\n", frame->InitializationErrorString().c_str());
            return false;
        }
        ++n;
    }

    return true;
}

void LogViewer::updateViews() {
    // Update current frame number
    QTime time = QTime::currentTime();
    if (!_lastUpdateTime.isNull()) {
        _doubleFrameNumber +=
            ui.playbackRate->value() * _lastUpdateTime.msecsTo(time) / 1000.0;
    }
    _lastUpdateTime = time;

    // Limit to available data
    _doubleFrameNumber = max(0.0, _doubleFrameNumber);
    _doubleFrameNumber = min(frames.size() - 1.0, _doubleFrameNumber);

    int f = frameNumber();
    const LogFrame& currentFrame = *frames[f];

    ui.timeSlider->setValue(f);

    // Copy recent history into the FieldView
    int n = min(f, (int)_history.size());
    for (int i = 0; i < n; ++i) {
        _history[i] = frames[f - i];
    }
    for (int i = n; i < (int)_history.size(); ++i) {
        _history[i].reset();
    }

    // Update non-message tree items
    _frameNumberItem->setData(ProtobufTree::Column_Value, Qt::DisplayRole,
                              frameNumber());
    int elapsedMillis =
        (currentFrame.command_time() - frames[0]->command_time() + 500) / 1000;
    QTime elapsedTime = QTime().addMSecs(elapsedMillis);
    _elapsedTimeItem->setText(ProtobufTree::Column_Value,
                              elapsedTime.toString("hh:mm:ss.zzz"));

    // Sort the tree by tag if items have been added
    if (ui.tree->message(currentFrame)) {
        // Items have been added, so sort again on tag number
        ui.tree->sortItems(ProtobufTree::Column_Tag, Qt::AscendingOrder);
    }

    ui.fieldView->update();
}

// Generates random alphanumeric strings
void LogViewer::generateMatchID(char* id) {
    string chars(
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890");

    // Avoid using time based srand init
    srand(getpid());

    for (int i = 0; i < MATCH_ID_LENGTH; ++i) {
        id[i] = chars[rand() % chars.length()];
    }

    id[MATCH_ID_LENGTH] = '\0';
}

void LogViewer::on_action0_triggered() { ui.fieldView->rotate(0); }

void LogViewer::on_action90_triggered() { ui.fieldView->rotate(1); }

void LogViewer::on_action180_triggered() { ui.fieldView->rotate(2); }

void LogViewer::on_action270_triggered() { ui.fieldView->rotate(3); }

void LogViewer::on_actionRawBalls_toggled(bool state) {
    ui.fieldView->showRawBalls = state;
    ui.fieldView->update();
}

void LogViewer::on_actionRawRobots_toggled(bool state) {
    ui.fieldView->showRawRobots = state;
    ui.fieldView->update();
}

void LogViewer::on_actionCoords_toggled(bool state) {
    ui.fieldView->showCoords = state;
    ui.fieldView->update();
}

void LogViewer::on_timeSlider_sliderPressed() { ui.playbackRate->setValue(0); }

void LogViewer::on_timeSlider_sliderMoved(int value) { frameNumber(value); }

void LogViewer::on_playbackRate_sliderReleased() {
    // Center the slider and stop playback
    ui.playbackRate->setValue(0);
}

void LogViewer::on_logNext_clicked() {
    ui.playbackRate->setValue(0);
    frameNumber(frameNumber() + 1);
}

void LogViewer::on_logPrev_clicked() {
    ui.playbackRate->setValue(0);
    frameNumber(frameNumber() - 1);
}

void LogViewer::on_logBeginning_clicked() { frameNumber(0); }

void LogViewer::on_logEnd_clicked() { frameNumber(frames.size() - 1); }
