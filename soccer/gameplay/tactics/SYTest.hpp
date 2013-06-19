/*
 * SYTest.hpp
 *
 *  Created on: Jan 23, 2013
 *      Author: Matthew Barulic
 */

#ifndef SYTEST_HPP_
#define SYTEST_HPP_

#include <gameplay/Play.hpp>
#include <gameplay/tactics/StableYank.hpp>

namespace Gameplay {
namespace Plays {

class SYTest: public Gameplay::Play {
public:
	SYTest(GameplayModule *gameplay);

	virtual bool run();

protected:
	StableYank _yanker;
};

} /* namespace Plays */
} /* namespace Gameplay */
#endif /* SYTEST_HPP_ */
