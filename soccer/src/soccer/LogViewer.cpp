#include <algorithm>
#include <cstdio>

#include <QApplication>
#include <QFile>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include <LogViewer.hpp>
#include <fcntl.h>

using namespace std;
using namespace boost;
using namespace Packet;
using namespace google::protobuf::io;

void usage(const char* prog) {
    fprintf(stderr, "Usage: %s <filename.log>\n", prog);
    exit(1);
}

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    if (argc != 2) {
        usage(argv[0]);
    }

    LogViewer win;

    win.readFrames(argv[1]);
    win.showMaximized();

    return QApplication::exec();
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

    QActionGroup* rotate_group = new QActionGroup(this);
    rotate_group->addAction(ui.action0);
    rotate_group->addAction(ui.action90);
    rotate_group->addAction(ui.action180);
    rotate_group->addAction(ui.action270);

    connect(&_updateTimer, SIGNAL(timeout()), SLOT(updateViews()));
    _updateTimer.start(30);
}

bool LogViewer::read_frames(const char* filename) {
    frames.clear();
    ui.timeSlider->setMaximum(0);

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

    ui.timeSlider->setMaximum(frames.size());
    return true;
}

void LogViewer::update_views() {
    // Update current frame number
    QTime time = QTime::currentTime();
    if (!_lastUpdateTime.isNull()) {
        _doubleFrameNumber += ui.playbackRate->value() * _lastUpdateTime.msecsTo(time) / 1000.0;
    }
    _lastUpdateTime = time;

    // Limit to available data
    _doubleFrameNumber = max(0.0, _doubleFrameNumber);
    _doubleFrameNumber = min(frames.size() - 1.0, _doubleFrameNumber);

    int f = frameNumber();
    const LogFrame& current_frame = *frames[f];

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
    _frameNumberItem->setData(ProtobufTree::Column_Value, Qt::DisplayRole, frameNumber());
    int elapsed_millis = (current_frame.command_time() - frames[0]->command_time() + 500) / 1000;
    QTime elapsed_time = QTime::fromMSecsSinceStartOfDay(elapsed_millis);
    _elapsedTimeItem->setText(ProtobufTree::Column_Value, elapsed_time.toString("hh:mm:ss.zzz"));

    // Sort the tree by tag if items have been added
    if (ui.tree->message(current_frame)) {
        // Items have been added, so sort again on tag number
        ui.tree->sortItems(ProtobufTree::Column_Tag, Qt::AscendingOrder);
    }

    ui.fieldView->update();
}

void LogViewer::on_action0_triggered() { ui.fieldView->rotate(0); }

void LogViewer::on_action90_triggered() { ui.fieldView->rotate(1); }

void LogViewer::on_action180_triggered() { ui.fieldView->rotate(2); }

void LogViewer::on_action270_triggered() { ui.fieldView->rotate(3); }

void LogViewer::on_action_raw_balls_toggled(bool state) {
    ui.fieldView->showRawBalls = state;
    ui.fieldView->update();
}

void LogViewer::on_action_raw_robots_toggled(bool state) {
    ui.fieldView->showRawRobots = state;
    ui.fieldView->update();
}

void LogViewer::on_action_coords_toggled(bool state) {
    ui.fieldView->showCoords = state;
    ui.fieldView->update();
}

void LogViewer::on_time_slider_slider_pressed() { ui.playbackRate->setValue(0); }

void LogViewer::on_time_slider_slider_moved(int value) { frameNumber(value); }

void LogViewer::on_playback_rate_slider_released() {
    // Center the slider and stop playback
    ui.playbackRate->setValue(0);
}

void LogViewer::on_log_next_clicked() {
    ui.playbackRate->setValue(0);
    frameNumber(frameNumber() + 1);
}

void LogViewer::on_log_prev_clicked() {
    ui.playbackRate->setValue(0);
    frameNumber(frameNumber() - 1);
}

void LogViewer::on_log_beginning_clicked() { frameNumber(0); }

void LogViewer::on_log_end_clicked() { frameNumber(frames.size() - 1); }
