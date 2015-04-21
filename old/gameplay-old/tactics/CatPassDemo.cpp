#include "CatPassDemo.hpp"
#include <gameplay/tactics/passing/StablePass.hpp>
#include <gameplay/tactics/passing/DumbReceive.hpp>
#include <iostream>


REGISTER_PLAY_CATEGORY(Gameplay::Plays::CatPassDemo, "Test")

Gameplay::Plays::CatPassDemo::CatPassDemo(GameplayModule *gameplay)
	: Play(gameplay),
	  passArray(),
	  currentPass(0)
{
	using namespace Geometry2d;

	PassReceiveTactic passTactic(gameplay);
	Point receivePoint;

	passTactic.setActions(new StablePass(gameplay), new DumbReceive(gameplay));
	passTactic.pointToReceive(Point(-1.0, 5.15));
	passArray.push_back(passTactic);

	passTactic.setActions(new StablePass(gameplay), new DumbReceive(gameplay));
	passTactic.pointToReceive(Point(1.0, 5.15));
	passArray.push_back(passTactic);

	passTactic.setActions(new StablePass(gameplay), new DumbReceive(gameplay));
	passTactic.pointToReceive(Point(-1.0, 3.8));
	passArray.push_back(passTactic);

	passTactic.setActions(new StablePass(gameplay), new DumbReceive(gameplay));
	passTactic.pointToReceive(Point(1.0, 3.8));
	passArray.push_back(passTactic);
}

Gameplay::Plays::CatPassDemo::~CatPassDemo()
{
	for(int i = 0; i < passArray.size(); i++) {
		delete passArray.at(i).pass();
		delete passArray.at(i).receive();
	}
}

bool Gameplay::Plays::CatPassDemo::initPositions()
{
	using namespace std;

	bool valid = true;
	for(int i = 0; i < passArray.size(); i++) {
		if(!passArray.at(i).positionsFilled()) {
			valid = false;
		}
	}
	if(valid)
		return true;

	std::vector<OurRobot*> robots;
	for (OurRobot* r :  gameplay()->playRobots()) {
		robots.push_back(r);
	}

	if(robots.size() <= 1) {
		return false;
	}

	int r_size = robots.size();
	int r_i = 0;

	int p_size = passArray.size();
	int p_i = 0;

	for(p_i = 0; p_i < p_size; p_i++) {
		PassReceiveTactic &tactic = passArray.at(p_i);

		int leader = r_i;
		int receiver = (r_i + 1) % r_size;

		tactic.setRobots(robots.at(leader), robots.at(receiver));

		r_i = receiver;
	}

	return true;
}

bool Gameplay::Plays::CatPassDemo::run()
{
	using namespace Geometry2d;

	if(!initPositions()) {
		return false;
	}

	PassReceiveTactic &tactic = passArray.at(currentPass);

	tactic.run();

	if(tactic.isDone()) {
		currentPass = (currentPass + 1) % passArray.size();
		tactic.restart();
	}

	return true;
}
