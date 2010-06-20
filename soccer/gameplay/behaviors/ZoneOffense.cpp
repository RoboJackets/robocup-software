#include "ZoneOffense.hpp"

#include <iostream>
#include <vector>
#include <boost/foreach.hpp>

using namespace std;

Gameplay::Behaviors::ZoneOffense::ZoneOffense(GameplayModule *gameplay)
: Behavior(gameplay, 2)
{
	_leftAttack = 0;
	_rightAttack = 0;
	_midfielder = 0;
}

bool Gameplay::Behaviors::ZoneOffense::assign(std::set<Robot *> &available) {
	takeAll(available);

	if (_robots.size() == 2) {
		Robot *a = *_robots.begin(), *b = *(_robots.begin()++);
		if (a->pos().x < b->pos().x) {
			_leftAttack = a;
			_rightAttack = b;
		} else {
			_leftAttack = b;
			_rightAttack = a;
		}
		return true;
	} else if (_robots.size() == 3) {

		// find the midfielder
		float backY = 100;
		vector<Robot*> forwards;
		BOOST_FOREACH(Robot * r, _robots) {
			if (!r) {
				_midfielder = r;
				backY = _midfielder->pos().y;
			} else if (r->pos().y < backY) {
				backY = r->pos().y;
				forwards.push_back(_midfielder);
				_midfielder = r;
			}
		}
		cout << "ZoneOffense: 3 robot version - found " << forwards.size() << " forwards" << endl;

		// pick left and right
		Robot *a = forwards[0], *b = forwards[1];
		if (a->pos().x < b->pos().x) {
			_leftAttack = a;
			_rightAttack = b;
		} else {
			_leftAttack = b;
			_rightAttack = a;
		}

		return true;
	}
	return false;
}

bool Gameplay::Behaviors::ZoneOffense::run() {
	return true;
}
