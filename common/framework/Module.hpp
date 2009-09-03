#pragma once

#include "SystemState.hpp"

#include <LogFrame.hpp>

#include <QString>
#include <QWidget>
#include <QPainter>
#include <QToolBar>

/** a module is called by the system trigger to process its data */
class Module
{
	public:
		virtual ~Module() {};
		
		/** sets the system state for the module */
		virtual void state(SystemState* state)
        {
            _state = state;
        }
        
        SystemState *state() const
        {
            return _state;
        }
		
		/** return a widget for the ui */
		virtual QWidget* widget() const { return _widget; }
		
		virtual QToolBar* toolbar() const { return _toolbar; }
		
		/** draw onto the field with the given painter */
		virtual void fieldOverlay(QPainter&, Packet::LogFrame&) const {}
		
		QString name() const { return _name; }
		
		/** run the code for the module */
		virtual void run() = 0;
		
		/** code run in a separate slower thread */
		virtual void slowRun() {};
		
		///handle house events on the field
		virtual void mousePress(QMouseEvent* me, Geometry2d::Point pos) {};
		virtual void mouseMove(QMouseEvent* me, Geometry2d::Point pos) {};
		virtual void mouseRelease(QMouseEvent* me, Geometry2d::Point pos) {};
		
		virtual void robotSelected(int id) { _selectedRobotId = id; }
		
	private:
		Module(Module&);
		Module& operator&=(Module&);
		
	protected:
		Module(QString name);
		
		/** this WILL BE SET before the module is run */
		SystemState* _state;
		
		QString _name;
		
		/** selected robot id */
		//-1 = no robot selected
		int _selectedRobotId;
		
		QToolBar* _toolbar;
		
		QWidget* _widget;
};
