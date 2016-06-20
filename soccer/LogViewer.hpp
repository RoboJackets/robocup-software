#pragma once

#include <ui_LogViewer.h>
#include <protobuf/LogFrame.pb.h>

#include <QTime>
#include <QTimer>
#include <vector>

class LogViewer : public QMainWindow {
    Q_OBJECT;

public:
    LogViewer(QWidget* parent = nullptr);

    int frameNumber() const { return roundf(_doubleFrameNumber); }

    void frameNumber(int value) { _doubleFrameNumber = value; }

    // This is called when
    bool readFrames(const char* filename);

    std::vector<std::shared_ptr<Packet::LogFrame> > frames;

public Q_SLOTS:
    void updateViews();

    void on_actionRawBalls_toggled(bool state);
    void on_actionRawRobots_toggled(bool state);
    void on_actionCoords_toggled(bool state);

    // Field rotation
    void on_action0_triggered();
    void on_action90_triggered();
    void on_action180_triggered();
    void on_action270_triggered();

    void on_timeSlider_sliderPressed();
    void on_timeSlider_sliderMoved(int value);
    void on_playbackRate_sliderReleased();
    void on_logBeginning_clicked();
    void on_logPrev_clicked();
    void on_logNext_clicked();
    void on_logEnd_clicked();

private:
    Ui_LogViewer ui;

    QTimer _updateTimer;
    QTime _lastUpdateTime;
    double _doubleFrameNumber;

    // Recent history.
    // Yeah, it's copied, but if it works in soccer then it works here.
    std::vector<std::shared_ptr<Packet::LogFrame> > _history;

    // Tree items that are not in LogFrame
    QTreeWidgetItem* _frameNumberItem;
    QTreeWidgetItem* _elapsedTimeItem;
};
