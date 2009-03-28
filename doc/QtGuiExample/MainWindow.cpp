#include "MainWindow.hpp"

MainWindow::MainWindow() :
	QMainWindow()
{
	//setup out window based on the designer gui
	setupUi(this);

	//standard style signal/slot connect
	connect(button2, SIGNAL(clicked()), SLOT (button2Click()));
}

MainWindow::~MainWindow()
{
	
}

void MainWindow::on_button1_clicked()
{
	label->setText("button 1");
}

void MainWindow::button2Click()
{
	label->setText("<strong>button 2</strong>");
}
