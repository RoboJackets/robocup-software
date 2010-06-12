#pragma once

#include <QWidget>
#include <framework/ConfigFile.hpp>

#include "ui_ConfigFileTab.h"

#include <configuration/ConfigFileModel.hpp>

class ConfigFileTab: public QWidget
{
	Q_OBJECT;

	public:
		ConfigFileTab(boost::shared_ptr<ConfigFile> config, QWidget *parent = 0);
		~ConfigFileTab(){}

		void save(QString filename);
		void load(QString filename);

	private:
		Ui_ConfigFileTab _ui;

		ConfigFileModel * _model;
};
