#ifndef _DISPLAY_HPP
#define _DISPLAY_HPP

#include <QGLWidget>
#include <QTimer>

#include <NxPhysics.h>

#include "Physics/Env.hpp"

class Viewer : public QGLWidget
{
	/// methods ///
	public:
		Viewer(Env* env, QWidget* parent = 0);
		~Viewer();

	protected:
		void initializeGL();
		void resizeGL(int w, int h);
		void paintGL();

	private:
		void renderData(const NxDebugRenderable& data) const;
		void setupColor(NxU32 color) const;

	/// members ///
	private:
		/** Timer for repainting the field display */
		QTimer _repaint;

		Env* _env;
};

#endif
