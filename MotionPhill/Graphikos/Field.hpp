#ifndef _GRAPHIKOS_FIELD_H_
#define _GRAPHIKOS_FIELD_H_

#include <QPainter>

//FIXME - Need to do something about the namespace conflict with constants

namespace Graphikos
{
	/**
	 * The Field class is responsible for drawing a field to a display
	 * It is called by the VisualizationWidget when the field needs to be drawn
	 */
	class Field
	{
		public:
			static void paint(QPainter& painter);

		private:
			Field();
			Field(Field&);

			/**
			 * draws the goals and related markings (penalty & arc)
			 * for the particular goal
			 * @param team team id to draw the goal for (0=yellow, 1=blue)
			 */
			static void paintGoal(unsigned char team, QPainter& painter);
	};
};

#endif
