#pragma once

#include "Planner.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"
#include <vector>

namespace Planning {
/**
 * @brief abstract Planner that conducts re-planning.
 * it's lazy because it only makes a new plan when the old plan is invalid, but
 * occasionally it will check for a better path even if the old path is valid.
 *
 * Derived classes just need to implement getGoalInstant()
 * other methods are provided with default implementations but can be overriden
 * no need to override plan()!
 *
 * all command types are converted to PathTargetCommands with the pathGoal
 * given by getGoalInstant(request)
 *
 * NOTE: call updatePrevTime(shell_id) whenever a new path is returned
 */
class LazyPlanner: public Planner {
public:
    LazyPlanner(): prevTimes(Num_Shells, RJ::now()-60s) {};
    virtual ~LazyPlanner() = default;

    Trajectory plan(PlanRequest&& request) override;

    std::string name() const override { return "LazyPlanner"; }
    static void createConfiguration(Configuration* cfg);

    /**
     * Specifies the duration of the previous path used during partial replan
     * @return duration
     */
    virtual RJ::Seconds partialReplanLeadTime() const {
        return RJ::Seconds{*_partialReplanLeadTime};
    }

protected:
    /**
     * get the angle function used to plan angles
     * @param request
     * @return angle function
     */
    virtual AngleFunction getAngleFunction(const PlanRequest& request) const;

    /**
     * create a new trajectory and discard the previous trajectory
     * @param request
     * @param angleFunction
     * @return trajectory
     */
    virtual Trajectory fullReplan(PlanRequest&& request, RobotInstant goalInstant, AngleFunction angleFunction) = 0;

    /**
     * use a small piece of the previous trajectory replan from that point forward
     * @param request
     * @param angleFunction
     * @return trajectory
     */
    virtual Trajectory partialReplan(PlanRequest&& request, RobotInstant goalInstant, AngleFunction angleFunction) = 0;

    /**
     * check for a better path even if the current path is valid.
     * @param request
     * @param angleFunction
     * @return better trajectory if one is found. otherwise previous trajectory
     */
    virtual Trajectory checkBetter(PlanRequest&& request, RobotInstant goalInstant, AngleFunction angleFunction) = 0;

    /**
     * reuse the previous trajectory
     * @param request
     * @return trajectory
     */
    virtual Trajectory reuse(PlanRequest&& request);

    /**
     * check if the goal changed enough to cause a replan
     * @param prevGoal
     * @param goal
     * @return true if the goal changed, false otherwise
     */
    virtual bool goalChanged(const RobotInstant& prevGoal, const RobotInstant& goal) const;

    /**
     * get the current goal instant given a plan request
     * @param request
     * @return goal instant (time stamp is undefined)
     */
    virtual RobotInstant getGoalInstant(const PlanRequest& request) = 0;

    virtual Trajectory partialPath(const Trajectory& prevTrajectory) const {
        //todo(Ethan) cut out old parts of the path?
        return prevTrajectory.subTrajectory(0s, (RJ::now() - prevTrajectory.begin_time()) + partialReplanLeadTime());
    }

    void updatePrevTime(unsigned int shell) {
        assert(shell >= 0 && shell < Num_Shells);
        prevTimes[shell] = RJ::now();
    }
    RJ::Time getPrevTime(unsigned int shell) const {
        assert(shell >= 0 && shell < Num_Shells);
        return prevTimes[shell];
    }

private:
    bool veeredOffPath(const PlanRequest& request) const;

    static ConfigDouble* _partialReplanLeadTime;
    std::vector<RJ::Time> prevTimes;


};

}