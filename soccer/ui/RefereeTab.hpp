#pragma once

#include <QWidget>
#include "ui_RefereeTab.h"

class RefereeTab : public QWidget {
    Q_OBJECT;

public:
    RefereeTab(QWidget* parent = nullptr);

protected Q_SLOTS:
    void on_externalReferee_toggled(bool value);

    void on_actionHalt_triggered();
    void on_actionStop_triggered();
    void on_actionReady_triggered();
    void on_actionForceStart_triggered();
    void on_actionKickoffBlue_triggered();
    void on_actionKickoffYellow_triggered();

    void on_refFirstHalf_clicked();
    void on_refOvertime1_clicked();
    void on_refHalftime_clicked();
    void on_refOvertime2_clicked();
    void on_refSecondHalf_clicked();
    void on_refPenaltyShootout_clicked();
    void on_refTimeoutBlue_clicked();
    void on_refTimeoutYellow_clicked();
    void on_refTimeoutEnd_clicked();
    void on_refTimeoutCancel_clicked();
    void on_refDirectBlue_clicked();
    void on_refDirectYellow_clicked();
    void on_refIndirectBlue_clicked();
    void on_refIndirectYellow_clicked();
    void on_refPenaltyBlue_clicked();
    void on_refPenaltyYellow_clicked();
    void on_refGoalBlue_clicked();
    void on_refSubtractGoalBlue_clicked();
    void on_refGoalYellow_clicked();
    void on_refSubtractGoalYellow_clicked();
    void on_refYellowCardBlue_clicked();
    void on_refYellowCardYellow_clicked();
    void on_refRedCardBlue_clicked();
    void on_refRedCardYellow_clicked();

protected:
    Ui_Referee ui;
};
