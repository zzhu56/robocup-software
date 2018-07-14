#include "CollectPathPlanner.hpp"
#include "CompositePath.hpp"
#include "MotionInstant.hpp"
#include <Configuration.hpp>

using namespace std;
using namespace Geometry2d;

namespace Planning {

REGISTER_CONFIGURABLE(SettlePathPlanner);

ConfigDouble* CollectPathPlanner::_ballSpeedPercentForDampen;
ConfigDouble* CollectPathPlanner::_minSpeedToIntercept;
ConfigDouble* CollectPathPlanner::_maxAngleOffBallForDampen;
ConfigDouble* CollectPathPlanner::_searchStartTime;
ConfigDouble* CollectPathPlanner::_searchEndTime;
ConfigDouble* CollectPathPlanner::_searchIncTime;

void CollectPathPlanner::createConfiguration(Configuration* cfg) {
    _ballSpeedPercentForDampen =
        new ConfigDouble(cfg, "CollectPathPlanner/ballSpeedPercentForDampen", 0.1); // %
    _minSpeedToIntercept =
        new ConfigDouble(cfg, "CollectPathPlanner/minSpeedToIntercept", 0.1); // m/s
    _maxAngleOffBallForDampen =
        new ConfigDouble(cfg, "CollectPathPlanner/maxAngleOffBallForDampen", 45); // Deg
    _searchStartTime=
        new ConfigDouble(cfg, "CollectPathPlanner/searchStartTime", 0.1); // Seconds
    _searchEndTime =
        new ConfigDouble(cfg, "CollectPathPlanner/searchEndTime", 6.0); // Seconds
    _searchIncTime =
        new ConfigDouble(cfg, "CollectPathPlanner/searchIncTime", 0.2); // Seconds
}

bool CollectPathPlanner::shouldReplan(const PlanRequest& planRequest) const {
    return true;
}

std::unique_ptr<Path> CollectPathPlanner::run(PlanRequest& planRequest) {
    const CollectCommand& command =
    dynamic_cast<const CollectCommand&>(*planRequest.motionCommand);

    // The direction we will try and bounce the ball when we dampen it to
    // speed up actions after capture
    targetFinalCaptureDirectionPos = command.target;

    // Start state for the specified robot
    const MotionInstant& startInstant = planRequest.start;
    // All the max velocity / acceleration constraints for translation / rotation
    const MotionConstraints& motionConstraints = planRequest.constraints.mot;
    const RobotConstraints& robotConstraints = planRequest.constraints;
    // List of obstacles
    Geometry2d::ShapeSet& obstacles = planRequest.obstacles;
    std::vector<DynamicObstacle>& dynamicObstacles = planRequest.dynamicObstacles;

    SystemState& systemState = planRequest.systemState;
    const Ball& ball = systemState.ball;

    const RJ::Time curTime = RJ::now();
}
}
