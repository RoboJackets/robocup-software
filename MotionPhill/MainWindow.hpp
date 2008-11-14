#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QMainWindow>
#include <QMouseEvent>
#include <QTimer>
#include <QPointF>
#include <ui_motion.h>
#include "FieldDisplay.hpp"
#include "RobotPath.hpp"

class MainWindow : public QMainWindow, Ui::MainWindow
{
	Q_OBJECT;

	public:
	    MainWindow(Team team);
	    ~MainWindow();
        public Q_SLOTS:
            void cursorPosition(float x, float y, float wx, float wy, QMouseEvent me);
            void redraw();
	private:
            Ui::MainWindow ui;

            RobotPath rp;
            QPointF mousePoint;
            QTimer _timer;

};

#endif // MAIN_WINDOW_HPP
