#pragma once

#include <QWidget>

#include <framework/ConfigFile.hpp>

#include "ui_ConfigFileTab.h"

class ConfigFileTab: public QWidget
{
	Q_OBJECT;

	public:
		ConfigFileTab(boost::shared_ptr<ConfigFile> config, QWidget *parent = 0);
		~ConfigFileTab(){}

		void save(QString filename);
		void load(QString filename);

	private Q_SLOTS:
		void on_loadConfig_clicked();
		void on_saveConfig_clicked();

		// TODO: add something for changes to table

	private:
		Ui_ConfigFileTab _ui;
		boost::shared_ptr<ConfigFile> _config;
};
