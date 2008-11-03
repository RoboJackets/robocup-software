#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QMainWindow>
#include <QMouseEvent>
#include <ui_motion.h>

#include "FieldDisplay.hpp"

class MainWindow : public QMainWindow, Ui::MainWindow
{
	Q_OBJECT;

	public:
	    MainWindow(Team team);
	    ~MainWindow();
        public Q_SLOTS:
            void cursorPosition(float x, float y, float wx, float wy, QMouseEvent me);
	private:
            Ui::MainWindow ui;

};

#endif // MAIN_WINDOW_HPP
