#ifndef REFEREE_HPP
#define REFEREE_HPP

#include <ui_Referee.h>
#include <QTimer>

class Referee : public QMainWindow, Ui::MainWindow
{
	Q_OBJECT;

	public:
		Referee();
		~Referee();

	private Q_SLOTS:
		/** send latest ref info */
		void send();
	
	private:
		QTimer _txTimer;

		/** number of ref packets to transmit in 1 second */
		static const int Hz = 10;
};

#endif // REFEREE_HPP
