#pragma once

#include <rj_protos/LogFrame.pb.h>
#include <ui_LogViewer.h>

#include <QTime>
#include <QTimer>
#include <vector>

class LogViewer : public QMainWindow {
    Q_OBJECT;

public:
    LogViewer(QWidget* parent = nullptr);

    int frame_number() const { return roundf(double_frame_number_); }

    void frame_number(int value) { double_frame_number_ = value; }

    // This is called when
    bool read_frames(const char* filename);

    std::vector<std::shared_ptr<Packet::LogFrame> > frames;

public Q_SLOTS:
    void update_views();

    void on_action_raw_balls_toggled(bool state);
    void on_action_raw_robots_toggled(bool state);
    void on_action_coords_toggled(bool state);

    // Field rotation
    void on_action0_triggered();
    void on_action90_triggered();
    void on_action180_triggered();
    void on_action270_triggered();

    void on_time_slider_slider_pressed();
    void on_time_slider_slider_moved(int value);
    void on_playback_rate_slider_released();
    void on_log_beginning_clicked();
    void on_log_prev_clicked();
    void on_log_next_clicked();
    void on_log_end_clicked();

private:
    Ui_LogViewer ui_;

    QTimer update_timer_;
    QTime last_update_time_;
    double double_frame_number_;

    // Recent history.
    // Yeah, it's copied, but if it works in soccer then it works here.
    std::vector<std::shared_ptr<Packet::LogFrame> > history_;

    // Tree items that are not in LogFrame
    QTreeWidgetItem* frame_number_item_;
    QTreeWidgetItem* elapsed_time_item_;
};
