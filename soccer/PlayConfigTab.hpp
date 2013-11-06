#pragma once

#include <vector>
#include <QWidget>

#include <boost/shared_ptr.hpp>

#include "ui_PlayConfigTab.h"

namespace Gameplay
{
	class GameplayModule;
};

class PlayConfigTab: public QWidget
{
	Q_OBJECT;
	
	public:
		PlayConfigTab(QWidget *parent = 0);
		
		// Called after GameplayModule is created to populate the list of available plays.
		void setup(std::shared_ptr<Gameplay::GameplayModule> gp);
		
		void load(QString filename);
		
		void enable(QString name);
		
		// Called periodically to update controls.
		// This is not actually called once per frame.
		void frameUpdate();
		
		void selectNone();
		
	private Q_SLOTS:
		void on_load_clicked();
		void on_save_clicked();
		void on_goalie_toggled(bool checked);
		void on_plays_itemChanged(QTreeWidgetItem *item);
		void on_plays_customContextMenuRequested(const QPoint &pos);
		
	private:
		Ui_PlayConfig ui;
		std::shared_ptr<Gameplay::GameplayModule> _gameplay;
		
		QIcon _iconRun;
		
		// Map from play name to tree item
		typedef QMap<QString, QTreeWidgetItem *> NameItemMap;
		NameItemMap _nameItemMap;
};
