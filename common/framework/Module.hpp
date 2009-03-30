#ifndef MODULE_HPP_
#define MODULE_HPP_

#include "SystemState.hpp"

#include <LogFrame.hpp>

#include <QString>
#include <QWidget>
#include <QPainter>

/** a module is called by the system trigger to process its data */
class Module
{
	public:
		virtual ~Module() {};
		
		/** sets the system state for the module */
		void setSystemState(SystemState* state);
		
		/** return a widget for the ui */
		virtual QWidget* widget() const { return _widget; }
		
		/** draw onto the field with the given painter */
		virtual void fieldOverlay(QPainter&, Packet::LogFrame&) const {}
		
		QString name() const { return _name; }
		
		/** run the code for the module */
		virtual void run() = 0;
		
		///handle house events on the field
		virtual void mousePress(QMouseEvent* me, Geometry::Point2d pos) {};
		virtual void mouseMove(QMouseEvent* me, Geometry::Point2d pos) {};
		virtual void mouseRelease(QMouseEvent* me, Geometry::Point2d pos) {};
		
	private:
		Module(Module&);
		Module& operator&=(Module&);
		
	protected:
		Module(QString name);
		
		/** this WILL BE SET before the module is run */
		SystemState* _state;
		
		QString _name;
		
		QWidget* _widget;
};

#endif /* MODULE_HPP_ */
