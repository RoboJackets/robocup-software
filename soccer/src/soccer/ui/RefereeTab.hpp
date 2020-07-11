#pragma once

#include <QWidget>
#include "ui_RefereeTab.h"

class RefereeTab : public QWidget {
    Q_OBJECT;

public:
    RefereeTab(QWidget* parent = nullptr);

protected Q_SLOTS:
    void on_externalReferee_toggled(bool value);

    static void on_actionHalt_triggered();
    static void on_actionStop_triggered();
    static void on_actionReady_triggered();
    static void on_actionForceStart_triggered();
    static void on_actionKickoffBlue_triggered();
    static void on_actionKickoffYellow_triggered();

    static void on_refFirstHalf_clicked();
    static void on_refOvertime1_clicked();
    static void on_refHalftime_clicked();
    static void on_refOvertime2_clicked();
    static void on_refSecondHalf_clicked();
    static void on_refPenaltyShootout_clicked();
    static void on_refTimeoutBlue_clicked();
    static void on_refTimeoutYellow_clicked();
    static void on_refTimeoutEnd_clicked();
    static void on_refTimeoutCancel_clicked();
    static void on_refDirectBlue_clicked();
    static void on_refDirectYellow_clicked();
    static void on_refIndirectBlue_clicked();
    static void on_refIndirectYellow_clicked();
    static void on_refPenaltyBlue_clicked();
    static void on_refPenaltyYellow_clicked();
    static void on_refGoalBlue_clicked();
    static void on_refSubtractGoalBlue_clicked();
    static void on_refGoalYellow_clicked();
    static void on_refSubtractGoalYellow_clicked();
    static void on_refYellowCardBlue_clicked();
    static void on_refYellowCardYellow_clicked();
    static void on_refRedCardBlue_clicked();
    static void on_refRedCardYellow_clicked();

protected:
    Ui_Referee ui{};
};
