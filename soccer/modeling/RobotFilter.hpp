#pragma once

#include <Robot.hpp>
#include <array>

#include <rc-fshare/robot_model.hpp>
#include <Eigen/Dense>

#include <Configuration.hpp>

/**
 * @brief An observation of a robot's position and angle at a certain time
 *
 * @details We get this data out of the packets that come from the visoin sytem.
 * This is fed through RobotFilter, which then sets the position,
 * angle, velocity, and angular velocity of the Robot.
 */
class RobotObservation {
public:
    RobotObservation()
        : pos(), angle(), time(), frameNumber(), valid(false), source() {}

    RobotObservation(Geometry2d::Point pos, float angle, RJ::Time time,
                     int frame, bool valid, int source)
        : pos(pos),
          angle(angle),
          time(time),
          frameNumber(frame),
          valid(valid),
          source(source) {}

    Geometry2d::Point pos;
    float angle;  /// in radians
    RJ::Time time;
    int frameNumber;
    bool valid;
    int source;

    // Compares the times on two observations.  Used for sorting.
    bool operator<(const RobotObservation& other) const {
        return time < other.time;
    }
};

/**
 * adjusts robot vision for accuracy and calculates velocities from positions
 */
class RobotFilter {
public:
    // Vision Related
    static constexpr int Num_Cameras = 4;

    // Encoder related
    static constexpr int Frame_Delay = 6;
    uint64_t last_rx_timestamp;
    std::vector<RobotModel::EncReading> enc_reading_buf;
    RobotModel::EncReading enc_reading_sum{};
    std::pair<Geometry2d::Point, double> enc_global_sum{};
    boost::circular_buffer<std::pair<Geometry2d::Point, double>> enc_global_buf{100};

    RobotFilter();

    /// Gives a new observation to the filter
    void update(const std::array<RobotObservation, Num_Cameras>& obs,
                RobotPose* robot, RJ::Time currentTime, uint32_t frameNumber,
                boost::optional<Packet::RadioRx> bots_latest_rx = boost::none,
                bool verbose = false);

    static void createConfiguration(Configuration* cfg);

private:
    /// Estimate for each camera
    RobotPose _estimates[Num_Cameras];
    RobotPose _currentEstimate;

    static ConfigDouble* _velocity_alpha;
    static ConfigInt* _vis_frame_delay;
};
