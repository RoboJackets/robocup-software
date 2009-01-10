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
            enum
            {
	       DrawingMode,
	       RunMode,
            };

	public:
	    MainWindow(Team team);
	    ~MainWindow();
        public Q_SLOTS:
            void redraw();
            void on_erase_clicked();
            void on_beizerCurve_clicked();
            void on_line_clicked();
            void on_erasePath_clicked();
            void on_run_clicked();
            void on_stop_clicked();
            void on_startPoint_clicked();
            void on_closePath_clicked();
            void on_arc_clicked();
            void on_close_clicked();

        protected:
//             void mouseMoveEvent(QMouseEvent* me);
	private:
            Ui::MainWindow ui;

            RobotPath* _rp;
            QPointF mousePoint;
            QTimer _timer;

            Team _team;

            FieldView* _fieldView;

            int _mode;
};

#endif // MAIN_WINDOW_HPP
