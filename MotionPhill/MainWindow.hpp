#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QMainWindow>
#include <ui_PhillMotion.h>

class MainWindow : public QMainWindow, Ui::MainWindow
{
	Q_OBJECT;

	public:
		MainWindow();
		~MainWindow();

	public Q_SLOTS:
		//shorthand method for creating a slot
		//on_<object>_<slot>
		void on_button1_clicked();

		void button2Click();

	private:
};

#endif // MAIN_WINDOW_HPP
