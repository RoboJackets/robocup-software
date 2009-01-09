#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include "FieldView.hpp"
#include "RobotPath.hpp"

#include <QMainWindow>
#include <QMouseEvent>
#include <QTimer>
#include <QPointF>
#include <ui_motion.h>

class MainWindow : public QMainWindow, Ui::MainWindow
{
	Q_OBJECT;

	public:
	    MainWindow(Team team);
	    ~MainWindow();
        public Q_SLOTS:
            void redraw();
            void on_point_clicked();
            void on_beizerCurve_clicked();
            void on_line_clicked();

        protected:
            void mouseMoveEvent(QMouseEvent* me);
	private:
            Ui::MainWindow ui;

            RobotPath* _rp;
            QPointF mousePoint;
            QTimer _timer;

            Team _team;

            FieldView* _fieldView;
};

#endif // MAIN_WINDOW_HPP
