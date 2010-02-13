/**
 *  GUI Testing play - enable just to show that drawing works
 */

#include <QColor>
#include "TestGUI.hpp"

using namespace std;

Gameplay::Plays::TestGUI::TestGUI(GameplayModule *gameplay):
	Play(gameplay)
{
}

bool Gameplay::Plays::TestGUI::run()
{
	// draw circles
	float rad = 0.2;
	Geometry2d::Point c1center = Geometry2d::Point(Constants::Field::Width/3, Constants::Field::Length*(0.5-0.3));
	Geometry2d::Point c2center = Geometry2d::Point(-Constants::Field::Width/3, Constants::Field::Length*(0.5-0.3));
	Geometry2d::Point c3center = Geometry2d::Point(Constants::Field::Width/3, Constants::Field::Length*(0.5+0.3));
	Geometry2d::Point c4center = Geometry2d::Point(-Constants::Field::Width/3, Constants::Field::Length*(0.5+0.3));

	drawCircle(c1center, rad, 0, 0, 255);
	drawCircle(c2center, rad, 255, 0, 0);
	drawCircle(c3center, rad, 0, 255, 255);
	drawCircle(c4center, rad, 155, 0, 255);

	// draw text
	drawText("Text1", c1center, 255, 0, 0);
	drawText("Text2", c2center, Qt::blue);
	drawText("Text3", c3center, Qt::blue);
	drawText("Text4", c4center, Qt::blue);

	return true;
}
