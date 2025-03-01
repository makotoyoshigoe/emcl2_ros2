// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "emcl2/ExpResetMcl2.h"

#include <rclcpp/rclcpp.hpp>

#include <stdlib.h>

#include <cmath>
#include <iostream>

namespace emcl2
{
ExpResetMcl2::ExpResetMcl2(
  const Pose & p, int num, const Scan & scan, const std::shared_ptr<OdomModel> & odom_model,
  const std::shared_ptr<LikelihoodFieldMap> & map, double alpha_th,
  double expansion_radius_position, double expansion_radius_orientation, double extraction_rate,
  double range_threshold, bool sensor_reset, 
  const GnssReset & odom_gnss, bool use_gnss_reset, bool use_wall_tracking, double gnss_reset_var, 
  double kld_th, double pf_var_th, 
  rclcpp_action::Client<WallTrackingAction>::SharedPtr wt_client, 
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr last_reset_gnss_pos_pub)
: Mcl::Mcl(p, num, scan, odom_model, map, odom_gnss),
  alpha_threshold_(alpha_th),
  expansion_radius_position_(expansion_radius_position),
  expansion_radius_orientation_(expansion_radius_orientation),
  extraction_rate_(extraction_rate),
  range_threshold_(range_threshold),
  sensor_reset_(sensor_reset), 
  use_gnss_reset_(use_gnss_reset), 
  use_wall_tracking_(use_wall_tracking),
  gnss_reset_var_(gnss_reset_var), 
  kld_th_(kld_th), 
  pf_var_th_(pf_var_th), 
  wt_client_(wt_client), 
  last_reset_gnss_pos_pub_(last_reset_gnss_pos_pub)
{
	// RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), 
	// "use_gnss_reset: %d, use_wall_tracking: %d, sqrt(gnss_reset_var): %lf, kld_th: %lf, pf_var_th: %lf", 
	// use_gnss_reset, use_wall_tracking, sqrt(gnss_reset_var_), kld_th_, pf_var_th_);
	wall_tracking_start_ = false;
	send_goal_options_ = rclcpp_action::Client<WallTrackingAction>::SendGoalOptions();
	send_goal_options_.goal_response_callback = std::bind(&ExpResetMcl2::goalResponseCallback, this, std::placeholders::_1);
	send_goal_options_.feedback_callback = std::bind(&ExpResetMcl2::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
	send_goal_options_.result_callback = std::bind(&ExpResetMcl2::resultCallback, this, std::placeholders::_1);
	last_reset_gnss_pos_.point.x = INFINITY;
	last_reset_gnss_pos_.point.y = INFINITY;
}

ExpResetMcl2::~ExpResetMcl2() {}

void ExpResetMcl2::goalResponseCallback(const GoalHandleWallTracking::SharedPtr & goal_handle)
{
    if (!goal_handle) {
		RCLCPP_ERROR(rclcpp::get_logger("emcl2"), "Goal was rejected by server");
	} else {
		RCLCPP_INFO(rclcpp::get_logger("emcl2"), "Goal accepted by server, waiting for result");
	}
}
void ExpResetMcl2::feedbackCallback(
	[[maybe_unused]] typename GoalHandleWallTracking::SharedPtr, 
	[[maybe_unused]] const std::shared_ptr<const typename WallTrackingAction::Feedback> feedback)
{
	pre_open_place_arrived_ = open_place_arrived_;
	open_place_arrived_ = feedback->open_place_arrived;
	if(!pre_open_place_arrived_ && open_place_arrived_) should_gnss_reset_ = true;
	if(alpha_ >= alpha_threshold_ && open_place_arrived_ && exec_reset_aft_wt_){
		RCLCPP_INFO(rclcpp::get_logger("emcl2"), "Send cancel goal to server");
		wt_client_->async_cancel_all_goals();
		last_reset_gnss_pos_.point.x = odom_gnss_.gnss_position_[0];
		last_reset_gnss_pos_.point.y = odom_gnss_.gnss_position_[1];
		last_reset_gnss_pos_.header.frame_id = "map";
		rclcpp::Clock clock;
		last_reset_gnss_pos_.header.stamp = clock.now();
		wall_tracking_start_ = false;
		last_reset_gnss_pos_pub_->publish(last_reset_gnss_pos_);
		exec_reset_aft_wt_ = false;
	}
}
void ExpResetMcl2::resultCallback(const GoalHandleWallTracking::WrappedResult & result)
{
	switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
	  	RCLCPP_ERROR(rclcpp::get_logger("emcl2"), "Goal was succeeded");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(rclcpp::get_logger("emcl2"), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(rclcpp::get_logger("emcl2"), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(rclcpp::get_logger("emcl2"), "Unknown result code");
        return;
    }
}

void ExpResetMcl2::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv)
{
	Scan scan;
	scan = scan_;

	scan.lidar_pose_x_ = lidar_x;
	scan.lidar_pose_y_ = lidar_y;
	scan.lidar_pose_yaw_ = lidar_t;

	double origin = inv ? scan.angle_max_ : scan.angle_min_;
	int sgn = inv ? -1 : 1;
	for(size_t i = 0; i < scan.ranges_.size() ; i++) {
		scan.directions_16bit_.push_back(Pose::get16bitRepresentation(
			origin + sgn * i * scan.angle_increment_));
	}

	double valid_pct = 0.0;
	int valid_beams = scan.countValidBeams(&valid_pct);
	if (valid_beams == 0) {
		return;
	}

	for (auto & p : particles_) {
		p.w_ *= p.likelihood(map_.get(), scan);
	}

	// RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "N0.0 particle weight is %lf. Sum weight is %lf.", particles_[0].w_, sum_w);
	alpha_ = nonPenetrationRate(static_cast<int>(particles_.size() * extraction_rate_), map_.get(), scan);
	// RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "alpha: %lf", alpha_);

	if (alpha_ < alpha_threshold_) {
		bool gnss_info_rel_is_low = tooFar() || odom_gnss_.isNAN();
		if(use_wall_tracking_ && gnss_info_rel_is_low){
			resetUseWallTracking(scan);
		} else if(use_gnss_reset_){
			gnssResetAndExpReset(scan);
		} else {
			expResetWithLLCalc(scan);
		}
	}

	if (normalizeBelief() > 0.000001) {
		resampling();
	} else {
		resetWeight();
	}

	processed_seq_ = scan_.seq_;
}

double ExpResetMcl2::nonPenetrationRate(int skip, LikelihoodFieldMap * map, Scan & scan)
{
	static uint16_t shift = 0;
	int counter = 0;
	int penetrating = 0;
	for (size_t i = shift % skip; i < particles_.size(); i += skip) {
		counter++;
		if (particles_[i].wallConflict(map, scan, range_threshold_, sensor_reset_)) {
			penetrating++;
		}
	}
	shift++;

	// std::cout << penetrating << " " << counter << std::endl;
	return static_cast<double>((counter - penetrating)) / counter;
}

bool ExpResetMcl2::tooFar()
{
	double distance = euclideanDistanceFromLastResetPos();
	// RCLCPP_INFO(rclcpp::get_logger("emcl2"), "Distance: %lf", distance);
	return (distance > 10.);
}

double ExpResetMcl2::euclideanDistanceFromLastResetPos()
{
	double dx = last_reset_gnss_pos_.point.x - odom_gnss_.gnss_position_[0];
	double dy = last_reset_gnss_pos_.point.y - odom_gnss_.gnss_position_[1];
	return hypot(dx, dy);
}

void ExpResetMcl2::resetUseWallTracking(Scan & scan)
{
	if(!wall_tracking_start_) sendWTGoal();
	if(should_gnss_reset_){
		// RCLCPP_INFO(rclcpp::get_logger("emcl2"), "Should GNSS Reset");
		exec_reset_aft_wt_ = true;
		gnssResetWithLLCalc(scan);
		should_gnss_reset_ = false;
	} else if(open_place_arrived_){
		gnssResetAndExpReset(scan);
		exec_reset_aft_wt_ = true;	
	}
}

void ExpResetMcl2::sendWTGoal()
{
	if(!wt_client_->wait_for_action_server()){
		RCLCPP_ERROR(rclcpp::get_logger("emcl2"), "Action server not available after waiting");
		rclcpp::shutdown();
	}
	auto goal_msg = WallTrackingAction::Goal();
	goal_msg.start = true;
	wt_client_->async_send_goal(goal_msg, send_goal_options_);
	RCLCPP_INFO(rclcpp::get_logger("emcl2"), "Send goal to server");
	wall_tracking_start_ = true;
}

void ExpResetMcl2::expansionReset(void)
{
	for (auto & p : particles_) {
		double length =
		  2 * (static_cast<double>(rand()) / RAND_MAX - 0.5) * expansion_radius_position_;
		double direction = 2 * (static_cast<double>(rand()) / RAND_MAX - 0.5) * M_PI;

		p.p_.x_ += length * cos(direction);
		p.p_.y_ += length * sin(direction);
		p.p_.t_ += 2 * (static_cast<double>(rand()) / RAND_MAX - 0.5) *
			   expansion_radius_orientation_;
		p.w_ = 1.0 / particles_.size();
	}
}

void ExpResetMcl2::expResetWithLLCalc(Scan & scan)
{
	RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "EXPANSION RESET");
	expansionReset();
	for (auto & p : particles_) {
		p.w_ *= p.likelihood(map_.get(), scan);
	}
}

void ExpResetMcl2::gnssResetWithLLCalc(Scan & scan)
{
	RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "GNSS RESET");
	odom_gnss_.setVariance(gnss_reset_var_, expansion_radius_position_*expansion_radius_position_);
	odom_gnss_.gnssReset(alpha_, alpha_threshold_, particles_, sqrt(gnss_reset_var_));
	for (auto & p : particles_) {
		p.w_ *= p.likelihood(map_.get(), scan);
	}
	// should_gnss_reset_ = false;
}

void ExpResetMcl2::gnssResetAndExpReset(Scan & scan)
{
	double kld = odom_gnss_.kld();
	// RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), 
	// 			"kld / kld_th: %lf / %lf, (x_var, y_var) / var_th: (%lf, %lf) / %lf", 
	// 			kld, kld_th_, odom_gnss_.pf_x_var_, odom_gnss_.pf_y_var_, pf_var_th_);
	bool kld_cond = kld < kld_th_;
	bool var_cond = odom_gnss_.pf_x_var_ < pf_var_th_ && odom_gnss_.pf_y_var_ < pf_var_th_;
	bool er_cond = kld_cond || var_cond;
	if(er_cond)	expResetWithLLCalc(scan);
	else gnssResetWithLLCalc(scan);
}

}  // namespace emcl2
