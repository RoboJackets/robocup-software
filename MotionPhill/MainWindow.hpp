#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include "FieldDisplay.hpp"
#include "RobotPath.hpp"

#include <QMainWindow>
#include <QMouseEvent>
#include <QTimer>
#include <QPointF>
#include <ui_motion.h>

#include <log/FieldView.hpp>

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

            RobotPath* _rp;
            QPointF mousePoint;
            QTimer _timer;

            Team _team;

            Log::FieldView* _fieldView;

};

#endif // MAIN_WINDOW_HPP
