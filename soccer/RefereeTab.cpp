#include "RefereeTab.hpp"
#include "RefereeTab.moc"

void command(char ch) {}

RefereeTab::RefereeTab(QWidget* parent) { ui.setupUi(this); }

void RefereeTab::on_externalReferee_toggled(bool value) {}

void RefereeTab::on_actionHalt_triggered() { command('H'); }

void RefereeTab::on_actionReady_triggered() { command(' '); }

void RefereeTab::on_actionStop_triggered() { command('S'); }

void RefereeTab::on_actionForceStart_triggered() { command('s'); }

void RefereeTab::on_refFirstHalf_clicked() { command('1'); }

void RefereeTab::on_refOvertime1_clicked() { command('o'); }

void RefereeTab::on_refHalftime_clicked() { command('h'); }

void RefereeTab::on_refOvertime2_clicked() { command('O'); }

void RefereeTab::on_refSecondHalf_clicked() { command('2'); }

void RefereeTab::on_refPenaltyShootout_clicked() { command('a'); }

void RefereeTab::on_refTimeoutBlue_clicked() { command('T'); }

void RefereeTab::on_refTimeoutYellow_clicked() { command('t'); }

void RefereeTab::on_refTimeoutEnd_clicked() { command('z'); }

void RefereeTab::on_refTimeoutCancel_clicked() { command('c'); }

void RefereeTab::on_actionKickoffBlue_triggered() { command('K'); }

void RefereeTab::on_actionKickoffYellow_triggered() { command('k'); }

void RefereeTab::on_refDirectBlue_clicked() { command('F'); }

void RefereeTab::on_refDirectYellow_clicked() { command('f'); }

void RefereeTab::on_refIndirectBlue_clicked() { command('I'); }

void RefereeTab::on_refIndirectYellow_clicked() { command('i'); }

void RefereeTab::on_refPenaltyBlue_clicked() { command('P'); }

void RefereeTab::on_refPenaltyYellow_clicked() { command('p'); }

void RefereeTab::on_refGoalBlue_clicked() {
    /*	if (_state->team == Blue)
        {
            ++_state->gameState.ourScore;
        } else {
            ++_state->gameState.theirScore;
        }*/
    command('G');
}

void RefereeTab::on_refSubtractGoalBlue_clicked() {
    /*	if (_state->team == Blue)
        {
            if (_state->gameState.ourScore)
            {
                --_state->gameState.ourScore;
            }
        } else {
            if (_state->gameState.theirScore)
            {
                --_state->gameState.theirScore;
            }
        }*/

    command('D');
}

void RefereeTab::on_refGoalYellow_clicked() {
    /*	if (_state->team == Blue)
        {
            ++_state->gameState.theirScore;
        } else {
            ++_state->gameState.ourScore;
        }*/

    command('g');
}

void RefereeTab::on_refSubtractGoalYellow_clicked() {
    /*	if (_state->team == Blue)
        {
            if (_state->gameState.theirScore)
            {
                --_state->gameState.theirScore;
            }
        } else {
            if (_state->gameState.ourScore)
            {
                --_state->gameState.ourScore;
            }
        }*/

    command('d');
}

void RefereeTab::on_refYellowCardBlue_clicked() { command('Y'); }

void RefereeTab::on_refYellowCardYellow_clicked() { command('y'); }

void RefereeTab::on_refRedCardBlue_clicked() { command('R'); }

void RefereeTab::on_refRedCardYellow_clicked() { command('r'); }
