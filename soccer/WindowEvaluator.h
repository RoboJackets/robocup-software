//
// Created by matt on 7/8/15.
//

#ifndef GT_ROBOJACKETS_ROBOCUP_WINDOWEVALUATOR_H
#define GT_ROBOJACKETS_ROBOCUP_WINDOWEVALUATOR_H

#include <boost/optional.hpp>
#include <Geometry2d/Segment.hpp>
#include <Geometry2d/Point.hpp>
#include "Robot.hpp"
#include "SystemState.hpp"

class Window {
public:
  Window();

  Window(double t0, double t1);

  double a0;
  double a1;
  double t0;
  double t1;
  Geometry2d::Segment segment;
};

using WindowingResult = std::pair<std::vector<Window>, boost::optional<Window>>;

class WindowEvaluator {
public:

  WindowEvaluator(SystemState *systemState);

  WindowingResult eval_pt_to_pt(Geometry2d::Point origin, Geometry2d::Point target);

  WindowingResult eval_pt_to_opp_goal(Geometry2d::Point origin);

  WindowingResult eval_pt_to_our_goal(Geometry2d::Point origin);

  void obstacle_range(std::vector<Window>& windows, double& t0, double& t1);

  void obstacle_robot(std::vector<Window>& windows, Geometry2d::Point origin, Geometry2d::Segment target, Geometry2d::Point bot_pos);

  WindowingResult eval_pt_to_seg(Geometry2d::Point origin, Geometry2d::Segment target);

  bool debug = false;
  bool chip_enabled = false;
  double max_chip_range = 0.3;
  double min_chip_range = 4.0;
  std::vector<Robot*> excluded_robots;
  std::vector<Geometry2d::Point> hypothetical_robot_locations;

private:
  SystemState *system;

};


#endif //GT_ROBOJACKETS_ROBOCUP_WINDOWEVALUATOR_H
