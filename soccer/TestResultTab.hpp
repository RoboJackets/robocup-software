#pragma once

#include <vector>
#include <QWidget>

#include <boost/shared_ptr.hpp>

#include "ui_TestResultTab.h"

namespace Gameplay
{
	class GameplayModule;
};

class TestResultTab: public QWidget
{
	Q_OBJECT;
	
	public:
		TestResultTab(QWidget *parent = 0);
		
		// Called after GameplayModule is created to populate the list of available things.
		void setup(boost::shared_ptr<Gameplay::GameplayModule> gp);
		
	private:
		Ui_TestResultTab ui;
		boost::shared_ptr<Gameplay::GameplayModule> _gameplay;
		
};
