#include "RobotFilter.hpp"
#include <Utils.hpp>
#include <iostream>

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
	log_file.open("enc_vis_filter.csv");

        log_file << "avg_pos.x() avg_pos.y() avg_ang "
                 << "_currentEstimate.pos.x() _currentEstimate.pos.y() _currentEstimate.angle "
                 << "enc[0] enc[1] enc[2] enc[3] "
                 << "enc_reading_sum[0] enc_reading_sum[1] enc_reading_sum[2] enc_reading_sum[3]"
                 << std::endl;
}

void RobotFilter::createConfiguration(Configuration* cfg) {
    _velocity_alpha = new ConfigDouble(cfg, "RobotFilter/Velocity_Alpha", 0.2);
    _vis_frame_delay = new ConfigInt(cfg, "RobotFilter/Vision_Frame_Delay", 6);
}

void RobotFilter::update(
    const std::array<RobotObservation, Num_Cameras>& observations,
    RobotPose* robot, RJ::Time currentTime, uint32_t frameNumber,
    boost::optional<Packet::RadioRx> bots_latest_rx) {

    bool have_valid_encs = false;

    // if we have rx, update rx sum and buffer
    if (bots_latest_rx && bots_latest_rx->timestamp() != last_rx_timestamp) {
        const auto& rx = *bots_latest_rx;
        //std::cout << "Have new rx in robot filter!" << rx.timestamp() << std::endl;
        last_rx_timestamp = rx.timestamp();

        // Check we have non zero frame delay, and packet has encoders
        if (*_vis_frame_delay != 0 && rx.encoders().size() == 4) {
            // if frame delay decreases, need to shrink encoder
            // buffer by removing older entries (beginning of vec)
            while (enc_reading_buf.size() >= *_vis_frame_delay) {
                enc_reading_sum -= *enc_reading_buf.begin();
                enc_reading_buf.erase(enc_reading_buf.begin());
            }

            // if frame delay increases, we'll just keep pushing back
            // without deleting history until we reach the needed size
            RobotModel::EncReading read;
            read << rx.encoders().Get(0),
                    rx.encoders().Get(1),
                    rx.encoders().Get(2),
                    rx.encoders().Get(3);

            enc_reading_buf.push_back(read);
            enc_reading_sum += read;

            have_valid_encs = true;
        }
    }

    bool anyValid =
        std::any_of(observations.begin(), observations.end(),
                    [](const RobotObservation& obs) { return obs.valid; });

    std::size_t pos_cnt = 0;
    Point avg_pos(0, 0);
    double avg_ang = 0;

    if (anyValid) {
        for (int i = 0; i < observations.size(); i++) {
            const auto& obs = observations[i];
            auto& estimate = _estimates[i];
            if (obs.valid && obs.pos.x() != 0) {
		avg_pos += obs.pos;
		avg_ang += obs.angle;
		pos_cnt++;

		/*
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
                } else if (robot->velValid) {set tabstop     =4
set softtabstop =4
set shiftwidth  =4
set expandtab
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
		*/
            }
        }

        avg_pos = avg_pos / pos_cnt;
	avg_ang = avg_ang / pos_cnt;



	_currentEstimate.vel = (_currentEstimate.pos - avg_pos) * 1/60.0f;
	_currentEstimate.pos = avg_pos;
	_currentEstimate.angle = avg_ang;
	_currentEstimate.velValid = true;


	/*
        _currentEstimate.pos = positionTotal / positionWeightTotal;
        if (velocityWeightTotal > 0) {
            _currentEstimate.vel = velocityTotal / velocityWeightTotal;
            _currentEstimate.velValid = true;
        } else {
            _currentEstimate.vel = Point();
            _currentEstimate.velValid = false;
        }
	*/

        _currentEstimate.visible = true;
        _currentEstimate.time = currentTime;
        _currentEstimate.visionFrame = frameNumber;

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

	    std::cout << "Enc read sum" << enc_reading_sum << std::endl;
            auto enc_delta_bdy_rel = RobotModel::get().EncToBot * enc_reading_sum.cast<double>();

            Point delta_bdy_rel_pos(enc_delta_bdy_rel[0,0], enc_delta_bdy_rel[1,0]);
            auto world_delta = delta_bdy_rel_pos.rotated(-M_PI / 2 + _currentEstimate.angle);
            // auto world_delta = delta_bdy_rel_pos.rotated(_currentEstimate.angle);

            _currentEstimate.pos += world_delta;
            _currentEstimate.angle += enc_delta_bdy_rel[2,0];

            if (enc_reading_buf.size() != 0) {
                log_file << avg_pos.x() << " " << avg_pos.y() << " " << avg_ang << " ";
                log_file << _currentEstimate.pos.x() << " " << _currentEstimate.pos.y() << " " << _currentEstimate.angle << " ";
	        auto enc = *(enc_reading_buf.end()-1);
                log_file << enc[0] << " " << enc[1] << " " << enc[2] << " " << enc[3] << " ";
                log_file << enc_reading_sum[0] << " " << enc_reading_sum[1] << " " << enc_reading_sum[2] << " " << enc_reading_sum[3];
                log_file << std::endl;
	}

            //std::cout << "Adding position delta: " << world_delta << std::endl;
        }

        *robot = _currentEstimate;
    } else {
        robot->visible = false;
    }
}
