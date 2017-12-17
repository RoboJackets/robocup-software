#pragma once
#include <vector>

/**
Class contained by Pid to automatically tune PID Coefficients
*/
class PidTuner {
public:

    /**
     *@brief Constructs a new PidTuner
     *@param ip The starting P value
     *@param ii The starting I value
     *@param id The starting D value
     *@param Sp The step used for P
     *@param Si The step used for I
     *@param Sd The step used for D
     */
    PidTuner(float ip, float ii, float id, float Sp = .1, float Si = .1, float Sd = .1);

    /** @brief Starts a cycle to test a PID value
     *  @details Call Once at the beginning of a test cycle
     */
    void startCycle();

    /** @brief Adds the error to the total score of the current PID set
     *  @details Call Once each frame during a PID test
     */
    void run(float err);

    /** @brief Ends the test of a PID value, returns True if more tuning is needed
     *  @details Call Once at the end of a test cycle
     */
    bool endCycle();

    float getP() { return _currentPid.p; }
    float getI() { return _currentPid.i; }
    float getD() { return _currentPid.d; }

private:

    //Struct to store P, I and D values and a score of the error
    struct PidSet {
        PidSet(float ip = 0, float ii = 0, float id = 0);
        inline bool operator==(const PidSet& pidIn) {
            return (p == pidIn.p) && (i == pidIn.i) && (d == pidIn.d);
        }
        inline bool operator<(const PidSet& pidIn) {
          return (score < pidIn.score);
        }
        float p;
        float i;
        float d;
        float score;
    };

    PidSet _initialPid;
    PidSet _currentPid;
    PidSet _bestPid;

    int _cycles;
    int _testNum;

    float _pScale;
    float _iScale;
    float _dScale;

    float _threshold;

    float _overScale;

    std::vector <PidSet> _testSets;
};
