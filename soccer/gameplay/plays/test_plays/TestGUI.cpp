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
	Packet::LogFrame::DebugCircle c1, c2, c3, c4;
	float rad = 0.8;
	c1.radius(rad);
	c2.radius(rad);
	c3.radius(rad);
	c4.radius(rad);
	c1.center = Geometry2d::Point(Constants::Field::Width/3, Constants::Field::Length*(0.5-0.3));
	c2.center = Geometry2d::Point(-Constants::Field::Width/3, Constants::Field::Length*(0.5-0.3));
	c3.center = Geometry2d::Point(Constants::Field::Width/3, Constants::Field::Length*(0.5+0.3));
	c4.center = Geometry2d::Point(-Constants::Field::Width/3, Constants::Field::Length*(0.5+0.3));

	// set colors
	c1.color[0] = 0; c1.color[1] = 0; c1.color[2] = 255;
	c2.color[0] = 255; c2.color[1] = 0; c2.color[2] = 0;
	c3.color[0] = 0; c3.color[1] = 255; c3.color[2] = 255;
	c4.color[0] = 155; c4.color[1] = 0; c4.color[2] = 255;

	gameplay()->state()->debugCircles.push_back(c1);
	gameplay()->state()->debugCircles.push_back(c2);
	gameplay()->state()->debugCircles.push_back(c3);
	gameplay()->state()->debugCircles.push_back(c4);


	return true;
}
