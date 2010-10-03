#include "TestResultTab.hpp"

#include <gameplay/GameplayModule.hpp>

TestResultTab::TestResultTab(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
}

void setup(boost::shared_ptr<Gameplay::GameplayModule> gp)
{
}


