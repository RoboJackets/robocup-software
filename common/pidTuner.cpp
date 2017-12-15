#include "pidTuner.hpp"
#include <iostream>
#include <math.h>
#include <algorithm>


PidTuner::PidTuner(float ip, float ii, float id, float Sp, float Si, float Sd) {
    _initialPid = PidSet(ip,ii,id);
    //_currentPid =  _initialPid;

    _currentPid = PidSet(3,0,0);

    _cycles = 0;
    _testNum = 0;

    _pScale = Sp;
    _iScale = Si;
    _dScale = Sd;

    _threshold = .0001;

    _overScale = 2; //the step is divided by overscale each time it overshoots the target
}

PidTuner::PidSet::PidSet(float ip, float ii, float id) {
    p=ip;
    i=ii;
    d=id;
    score=0;
}

void PidTuner::startCycle() {
    //if old test set is done, determine new values to test
    if(_testNum==_testSets.size()) {
        _testSets.clear();
        //maintain the score for this since it has been tested before
        _testSets.push_back(_currentPid);

        _testSets.push_back(PidSet(_currentPid.p + _pScale, _currentPid.i, _currentPid.d));
        if(_currentPid.p - _pScale >= 0) {
            _testSets.push_back(PidSet(_currentPid.p - _pScale, _currentPid.i, _currentPid.d));
        }

        _testSets.push_back(PidSet(_currentPid.p, _currentPid.i + _iScale, _currentPid.d));
        if( _currentPid.i - _iScale > 0) {
            _testSets.push_back(PidSet(_currentPid.p, _currentPid.i - _iScale, _currentPid.d));
        }

        _testSets.push_back(PidSet(_currentPid.p, _currentPid.i, _currentPid.d + _dScale));
        if( _currentPid.d - _dScale > 0) {
            _testSets.push_back(PidSet(_currentPid.p, _currentPid.i, _currentPid.d - _dScale));
        }

        //don't test the first value if we have a score for it already
        _testNum = _cycles ? 1 : 0;
    }
    _currentPid = _testSets[_testNum];
}

void PidTuner::run(float err) {
    _currentPid.score += fabs(err);
}

//returns if pid needs more tuning, if it does, sets up next test points
bool PidTuner::endCycle() {
    _testSets[_testNum] = _currentPid;
    _testNum += 1;

    std::cout<<_currentPid.p<<" | "<<_currentPid.i<<" | "<<_currentPid.d<<std::endl;
    std::cout<<_currentPid.score<<std::endl;

    if(_testNum==_testSets.size()) {
        std::cout<<"---------------------------"<<std::endl;

        //finished testing set, determine best pid, then check if we need more tests
        _cycles += 1;

        if(_cycles > 1){
            //if we have found a _bestPid, check it too
            _testSets.push_back(_bestPid);
        }
        else{
            //stand in
            _testSets.push_back(_testSets[0]);
        }

        //Find the set with the lowest score
        _bestPid = *(std::min_element(std::begin(_testSets),std::end(_testSets)));

        //Don't actually need to include bestPid in testSet
        _testSets.pop_back();

        //Scales the test step down and reverses the direction
        //_pScale *= _testSets[0].score > _testSets[1].score ? 1 : -1/_overScale;
        //_iScale *= _testSets[0].score > _testSets[2].score ? 1 : -1/_overScale;
        //_dScale *= _testSets[0].score > _testSets[3].score ? 1 : -1/_overScale;

        _currentPid = _bestPid;

        std::cout<<"Best: "<<_bestPid.p<<" | "<<_bestPid.i<<" | "<<_bestPid.d<<" | "<<_bestPid.score<<std::endl;
        std::cout<<"---------------------------"<<std::endl;

        //return if we should keep testing
        return fabs(_pScale) >= _threshold;

    } else {
        return true; //keep testing
    }
}
