#pragma once

#include "RRTPlanner.hpp"
#include "SingleRobotPathPlanner.hpp"
class Configuration;
class ConfigDouble;

namespace Planning {

/**
 * @brief Planner which tries to slowly move in and control the ball at a short distance
 *
 * TODO: Clean up description
 */
class CollectPathPlanner : public SingleRobotPathPlanner {
public:
    CollectPathPlanner() : SingleRobotPathPlanner(false), rrtPlanner(0, 250){};
    virtual std::unique_ptr<Path> run(PlanRequest& planRequest) override;

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::Collect;
    }

    static void createConfiguration(Configuration* cfg);

private:
    bool shouldReplan(const PlanRequest& planRequest) const;

    RRTPlanner rrtPlanner;
    boost::optional<Geometry2d::Point> targetFinalCaptureDirectionPos;

    static ConfigDouble* _ballSpeedPercentForDampen;
    static ConfigDouble* _minSpeedToIntercept;
    static ConfigDouble* _maxAngleOffBallForDampen;
    static ConfigDouble* _searchStartTime;
    static ConfigDouble* _searchEndTime;
    static ConfigDouble* _searchIncTime;
};
}
