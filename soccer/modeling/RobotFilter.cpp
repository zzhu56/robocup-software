#include "RobotFilter.hpp"
#include <Utils.hpp>
#include <iostream>
#include <boost/circular_buffer.hpp>

using namespace std;
using namespace Geometry2d;

// How long to coast a robot's position when it isn't visible
static const RJ::Seconds Coast_Time(0.8);
static const RJ::Seconds Min_Frame_Time(0.014);
static const RJ::Seconds Min_Velocity_Valid_Time(0.1);
static const RJ::Seconds Vision_Timeout_Time(0.25);
static const RJ::Seconds Min_Double_Packet_Time(1.0 / 120);

REGISTER_CONFIGURABLE(RobotFilter);

ConfigDouble* RobotFilter::_velocity_alpha;
ConfigInt* RobotFilter::_vis_frame_delay;

RobotFilter::RobotFilter()
    : last_rx_timestamp(0)
{
        enc_reading_sum << 0, 0, 0, 0;
}

void RobotFilter::createConfiguration(Configuration* cfg) {
    _velocity_alpha = new ConfigDouble(cfg, "RobotFilter/Velocity_Alpha", 0.2);
    _vis_frame_delay = new ConfigInt(cfg, "RobotFilter/Vision_Frame_Delay", 6);
}

void RobotFilter::update(
    const std::array<RobotObservation, Num_Cameras>& observations,
    RobotPose* robot, RJ::Time currentTime, uint32_t frameNumber,
    boost::optional<Packet::RadioRx> bots_latest_rx,
    bool verbose) {

    if (verbose) std::cout << "====================" << std::endl;
    if (verbose) std::cout << "Frame: " << frameNumber << std::endl;
    if (verbose && bots_latest_rx) std::cout << "Has RX" << std::endl;

    bool have_valid_encs = true;

    if (verbose) {
//        std::cout << "RX timestamp: " << bots_latest_rx->timestamp() << std::endl;
//        std::cout << "Last timestamp: " << last_rx_timestamp << std::endl;
    }

    // if we have rx, update rx sum and buffer
    if (bots_latest_rx && bots_latest_rx->timestamp() != last_rx_timestamp) {
        if (verbose) std::cout << "Appending to encoder list" << std::endl;
        const auto& rx = *bots_latest_rx;
        const auto timeDiff = RJ::numSeconds(last_rx_timestamp - rx.timestamp());
        last_rx_timestamp = rx.timestamp();

        // Check we have non zero frame delay, and packet has encoders
        if (*_vis_frame_delay != 0 && rx.encoders().size() == 4) {
            // if frame delay decreases, need to shrink encoder
            // buffer by removing older entries (beginning of vec)
            while (enc_reading_buf.size() >= *_vis_frame_delay) {
                enc_reading_buf.erase(enc_reading_buf.begin());

                const auto &first = *enc_global_buf.begin();
                enc_global_sum.first -= first.first;
                enc_global_sum.second -= first.second;
                enc_global_buf.erase(enc_global_buf.begin());

            }

            // if frame delay increases, we'll just keep pushing back
            // without deleting history until we reach the needed size
            RobotModel::EncReading read;
            read << rx.encoders().Get(0).value(),
                    rx.encoders().Get(1).value(),
                    rx.encoders().Get(2).value(),
                    rx.encoders().Get(3).value();
//
//            cout << rx.robot_id()<<"encoders";
//
//
//            cout << rx.encoders().Get(0).value()<< " " <<
//                    rx.encoders().Get(1).value()<< " " <<
//                    rx.encoders().Get(2).value()<< " " <<
//                    rx.encoders().Get(3).value() <<endl;
//
//
//            cout << "sum" << enc_reading_sum<<endl;

            enc_reading_buf.push_back(read);
            enc_reading_sum += read;

            auto enc_delta_bdy_rel = RobotModel::get().EncToBot * read.cast<double>();
            Point delta_bdy_rel_pos(enc_delta_bdy_rel[0,0], enc_delta_bdy_rel[1,0]);

            const auto &delta_ang = enc_delta_bdy_rel[2,0];
            enc_global_sum.second += delta_ang;

            auto estimate_angle = _currentEstimate.angle + enc_global_sum.second;
            auto world_delta = delta_bdy_rel_pos.rotated(-M_PI / 2 + estimate_angle);
            enc_global_sum.first += world_delta;

            enc_global_buf.push_back(make_pair(world_delta, delta_ang));

            have_valid_encs = true;
        }
    }

    bool anyValid =
        std::any_of(observations.begin(), observations.end(),
                    [](const RobotObservation& obs) { return obs.valid; });
    if (anyValid) {
        if (verbose) std::cout << "Updating vision" << std::endl;
        for (int i = 0; i < observations.size(); i++) {
            const auto& obs = observations[i];
            auto& estimate = _estimates[i];
            if (obs.valid) {
                Point velEstimate{};
                double angleVelEstimate = 0;

                const auto dtime = RJ::Seconds(obs.time - estimate.time);
                if (dtime < Min_Double_Packet_Time) {
                    // If we got two packets too quickly, assume the latest one
                    // is correct
                    velEstimate = estimate.vel;
                    angleVelEstimate = estimate.angleVel;
                    estimate.velValid = true;
                } else if (dtime < Min_Velocity_Valid_Time) {
                    // If we got two packets at an expected time, properly
                    // calculate vel and angle
                    velEstimate = (obs.pos - estimate.pos) / dtime.count();
                    angleVelEstimate =
                        fixAngleRadians(obs.angle - estimate.angle) /
                        dtime.count();
                    estimate.velValid = true;
                } else if (robot->velValid) {
                    velEstimate = robot->vel;
                    angleVelEstimate = robot->angleVel;
                }

                // velocity alpha is the amount to 'trust' new data by
                const auto velocityAlpha = *_velocity_alpha;
                if (dtime < Min_Velocity_Valid_Time && estimate.velValid) {
                    // Weight old data and new data by 'velocityAlpha'
                    estimate.vel = velEstimate * velocityAlpha +
                                   estimate.vel * (1.0f - velocityAlpha);
                    estimate.angleVel = fixAngleRadians(
                        angleVelEstimate * velocityAlpha +
                        estimate.angleVel * (1.0f - velocityAlpha));
                } else {
                    estimate.vel = velEstimate;
                    estimate.angleVel = fixAngleRadians(angleVelEstimate);
                }

                estimate.pos = obs.pos;
                estimate.angle = obs.angle;
                estimate.visible = true;
                estimate.time = obs.time;
                estimate.visionFrame = obs.frameNumber;
            }
        }

        Point positionTotal{};
        double positionWeightTotal = 0;

        Point velocityTotal{};
        double velocityWeightTotal = 0;

        double angleTotal = 0;
        double angleWeightTotal = 0;

        double angleVelTotal = 0;
        double angleVelWeightTotal = 0;

        // Weight observations based on time since we've seen and average
        // everything together.
        for (const auto& estimate : _estimates) {
            const auto dTime = RJ::Seconds(currentTime - estimate.time);

            if (estimate.visible && dTime < Vision_Timeout_Time) {
                Point pos{};
                double angle{};
                // treat data with less certainty the older it is
                double currentPosWeight = std::max(
                    0.0, 1.0 - std::pow(dTime / Vision_Timeout_Time, 2));
                if (estimate.velValid) {
                    pos = estimate.pos + estimate.vel * dTime.count();
                    velocityTotal += estimate.vel * currentPosWeight;
                    velocityWeightTotal += currentPosWeight;

                    angle = estimate.angle + estimate.angleVel * dTime.count();
                    angleVelTotal += estimate.angleVel * currentPosWeight;
                    angleVelWeightTotal += currentPosWeight;
                } else {
                    pos = estimate.pos;

                    angle = estimate.angle;
                    currentPosWeight /= 2;
                }
                positionTotal += pos * currentPosWeight;
                positionWeightTotal += currentPosWeight;

                angleTotal += angle * currentPosWeight;
                angleWeightTotal += currentPosWeight;
            }
        }

        _currentEstimate.pos = positionTotal / positionWeightTotal;
        if (velocityWeightTotal > 0) {
            _currentEstimate.vel = velocityTotal / velocityWeightTotal;
            _currentEstimate.velValid = true;
        } else {
            _currentEstimate.vel = Point();
            _currentEstimate.velValid = false;
        }

        _currentEstimate.visible = true;
        _currentEstimate.time = currentTime;
        _currentEstimate.visionFrame = frameNumber;
        _currentEstimate.angle = angleTotal / angleWeightTotal;

        if (angleVelWeightTotal > 0) {
            _currentEstimate.angleVel = angleVelTotal / angleVelWeightTotal;
        } else {
            _currentEstimate.angleVel = 0;
        }
    }

    if (currentTime - _currentEstimate.time < Vision_Timeout_Time) {
        // hacking in encoder readings here, this may need to move
        // _currentEstimate.angle, vision estimated angle
        // _currentEstimate.vel, vision estimated velocity
        // _currentEstimate.pos, vision estimate pos
        // encoders will only provide a possibly better position/angle estimate

        if (have_valid_encs) {
            Eigen::Matrix<double, 3, 1> vision_pos;
            vision_pos << _currentEstimate.pos.x(),
                          _currentEstimate.pos.y(),
                          _currentEstimate.angle;


//            auto enc_delta_bdy_rel = RobotModel::get().EncToBot * enc_reading_sum.cast<double>();

//            Point delta_bdy_rel_pos(enc_delta_bdy_rel[0,0], enc_delta_bdy_rel[1,0]);
//            auto world_delta = delta_bdy_rel_pos.rotated(-M_PI / 2 + _currentEstimate.angle);


            _currentEstimate.pos += enc_global_sum.first;
            _currentEstimate.angle += enc_global_sum.second;
//            _currentEstimate.pos += world_delta;
//            _currentEstimate.angle += enc_delta_bdy_rel[2,0];

            if (verbose) std::cout << "Adding position deltas" << std::endl;
            std::stringstream ss;
            cout << enc_global_sum.first << " " << enc_global_sum.second << endl;


            auto our_robot = static_cast<OurRobot*>(robot);
//            our_robot->addText(QString::fromStdString(ss.str()), Qt::white, "positions");
        }

        *robot = _currentEstimate;
    } else {
        robot->visible = false;
    }
}
