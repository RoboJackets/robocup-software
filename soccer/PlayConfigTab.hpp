#pragma once

#include <vector>
#include <QWidget>

#include <boost/shared_ptr.hpp>

#include "ui_PlayConfigTab.h"

namespace Gameplay
{
	class Play;
	class GameplayModule;
};

class PlayListModel;

class PlayConfigTab: public QWidget
{
	Q_OBJECT;
	
	public:
		PlayConfigTab(QWidget *parent = 0);
		~PlayConfigTab();
		
		void addPlay(boost::shared_ptr<Gameplay::Play> play);
		
		Gameplay::GameplayModule *gameplay;
		
		void load(QString filename);
		
	private Q_SLOTS:
		void on_load_clicked();
		void on_save_clicked();
		void on_all_clicked();
		void on_none_clicked();
		void on_goalie_toggled(bool checked);
		
	private:
		Ui_PlayConfig ui;
		PlayListModel *_model;
};
