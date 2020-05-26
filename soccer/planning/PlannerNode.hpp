#pragma once

#include <Context.hpp>
#include <Node.hpp>

namespace planning {

class PlannerNode : public Node {
public:
    PlannerNode(Context* context);
    void run() override;

private:
    Context* context_;
};
}  // namespace planning
