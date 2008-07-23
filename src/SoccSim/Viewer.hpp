#ifndef _DISPLAY_HPP
#define _DISPLAY_HPP

#include <QWidget>
#include <QGLWidget>
#include <QTimer>

#include "Physics/Env.hpp"

class Viewer : public QGLWidget
{
	/// methods ///
	public:
		Viewer(QWidget* parent = 0);
		~Viewer();
		
		/** set the environment to display */
		void setEnv(Physics::Env* env);
		
	protected:
		void initializeGL();
		void resizeGL(int w, int h);
		void paintGL();
		
	/// members ///
	private:
		/** Simulated evironment */
		Physics::Env* _env;
		
		/** Timer for repainting the field display */
		QTimer _repaint;
};

#endif
