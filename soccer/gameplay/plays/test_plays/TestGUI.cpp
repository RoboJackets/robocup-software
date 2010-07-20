/**
 *  GUI Testing play - enable just to show that drawing works
 */

#include <QColor>
#include "TestGUI.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TestGUI, "Tests")

Gameplay::Plays::TestGUI::TestGUI(GameplayModule *gameplay):
	Play(gameplay, 0)
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

	state()->drawCircle(c1center, rad, Qt::blue);
	state()->drawCircle(c2center, rad, Qt::red);
	state()->drawCircle(c3center, rad, Qt::cyan);
	state()->drawCircle(c4center, rad, Qt::magenta);

	// draw text
	state()->drawText("Text1", c1center, Qt::red);
	state()->drawText("Text2", c2center, Qt::blue);
	state()->drawText("Text3", c3center, Qt::blue);
	state()->drawText("Text4", c4center, Qt::blue);

	return true;
}
