#ifndef JOGDIAL_HPP_
#define JOGDIAL_HPP_

#include <QWidget>

class JogDial : public QWidget
{
	Q_OBJECT;
	
	public:
		JogDial(QWidget* parent = 0);
		
		/** returns the normalized offset */
		float offset() const { return _offset*2.0f/_viewSpan; }
		
	Q_SIGNALS:
		/** emitted when the offset changes, normalized */
		void valueChanged(float offset);
		
	protected:
		void paintEvent(QPaintEvent* pe);
		void mousePressEvent(QMouseEvent* me);
		void mouseMoveEvent(QMouseEvent* me);
		void mouseReleaseEvent(QMouseEvent* me);
		
	private:
		/** angle offset from start (not normalized)*/
		float _offset;
		
		/** degrees between dial marks */
		float _spacing;
		
		/** range of angles to see */
		float _viewSpan;
		
		/** start x coordinate for mouse drag */
		int _x;
};

#endif /* JOGDIAL_HPP_ */
