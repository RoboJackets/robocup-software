#ifndef FIELDVIEW_HPP_
#define FIELDVIEW_HPP_

#include <QWidget>

#include <Team.h>
#include <LogFrame.hpp>

/** class that performs drawing of log data onto the field */
class FieldView : public QWidget
{
	Q_OBJECT;

	public:
		FieldView(Team t, QWidget* parent = 0);

	protected:
		void paintEvent(QPaintEvent* pe);
		void resizeEvent(QResizeEvent* re);
		//void mouseReleaseEvent(QMouseEvent* me);

	public Q_SLOTS:
		void frame(Packet::LogFrame* frame);

	private:
		/** frame to display */
		Packet::LogFrame* _frame;

		//translations for placing robots in team space
		float _tx, _ty, _ta;

		Team _team;
};

#endif /* FIELDVIEW_HPP_ */
