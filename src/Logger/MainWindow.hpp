#ifndef MAINWINDOW_HPP_
#define MAINWINDOW_HPP_

#include <QMainWindow>
#include <QTimer>

#include <ui_MainWindow.h>

#include "FieldDisplay.hpp"
#include "Log/Logger.hpp"
#include "PlaybackControl.hpp"

#include <Packet/VisionData.hpp>

class MainWindow : public QMainWindow
{
	Q_OBJECT;

	public:
		MainWindow(Team team);
		~MainWindow();

		void loggable(Log::Loggable* loggable);

	private Q_SLOTS:
		void redraw();

	private:
		//adds a display widget to the available tools
		void addDisplay(QString name, QWidget* w);

		Ui::MainWindow _ui;

		/** field display drawer */
		FieldDisplay _disp;

		/** displays update timer */
		QTimer _timer;

		/** displays widget */
		QWidget* _typeConfigs;

		/** receives packets and processes them for logging
		 *  Can be asked for a list of packets to show */
		Log::Logger _logger;

		PlaybackControl _control;
		
		/** latest vision information */
		Packet::VisionData _vision;
};

#endif /*MAINWINDOW_HPP_*/
