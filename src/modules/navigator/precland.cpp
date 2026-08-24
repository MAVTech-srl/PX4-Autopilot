/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @file precland.cpp
 *
 * Helper class to do precision landing with a landing target
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#include "precland.h"
#include "navigator.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

#define SEC2USEC 1000000.0f

static constexpr const char *LOST_TARGET_ERROR_MESSAGE = "Lost landing target while landing";

PrecLand::PrecLand(Navigator *navigator) :
	MissionBlock(navigator, vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND),
	ModuleParams(navigator)
{
	updateParams();
}

void
PrecLand::on_activation()
{
	_state = PrecLandState::Start;
	_search_cnt = 0;
	_target_pose = {};
	_target_pose_valid = false;
	_target_pose_updated = false;
	_have_target_ever = false;
	_target_velocity.zero();
	_consecutive_outlier_rejections = 0;
	_target_yaw_valid = false;
	_target_yaw = 0.f;
	_last_yaw_update = 0;
	_hover_settle_time = 0;
	_last_offset_update = hrt_absolute_time();
	_current_offset_z = -_param_pld_hov_alt.get();
	_search_pattern_start_time = 0;
	_search_return_start_time = 0;
	_search_center.zero();
	_search_altitude_z = 0.f;
	_search_phase = PrecLandSearchPhase::Climb;

	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	if (!_map_ref.isInitialized()) {
		_map_ref.initReference(vehicle_local_position->ref_lat, vehicle_local_position->ref_lon, hrt_absolute_time());
	}

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	pos_sp_triplet->next.valid = false;
	pos_sp_triplet->previous.valid = false;

	// Check that the current position setpoint is valid, otherwise land at current position
	if (!pos_sp_triplet->current.valid) {
		PX4_WARN("Reset");
		pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
		pos_sp_triplet->current.valid = true;
		pos_sp_triplet->current.timestamp = hrt_absolute_time();
	}

	switch_to_state_start();

	_is_activated = true;
}

void
PrecLand::on_active()
{
	_target_pose_updated = update_target_estimate();
	const uint64_t now = hrt_absolute_time();
	_target_pose_valid = _have_target_ever && target_age_s(now) <= _param_pld_btout.get();

	// stop if we are landed
	if (_navigator->get_land_detected()->landed) {
		switch_to_state_done();
	}

	switch (_state) {
	case PrecLandState::Start:
		run_state_start();
		break;

	case PrecLandState::HorizontalApproach:
		run_state_horizontal_approach();
		break;

	case PrecLandState::DescendAboveTarget:
		run_state_descend_above_target();
		break;

	case PrecLandState::FinalApproach:
		run_state_final_approach();
		break;

	case PrecLandState::Search:
		run_state_search();
		break;

	case PrecLandState::Fallback:
		run_state_fallback();
		break;

	case PrecLandState::Done:
		// nothing to do
		break;

	default:
		// unknown state
		break;
	}

	publish_status();
}

void
PrecLand::on_inactivation()
{
	_is_activated = false;
}

void
PrecLand::updateParams()
{
	ModuleParams::updateParams();
}

void
PrecLand::run_state_start()
{
	// check if target visible and go to horizontal approach
	if (switch_to_state_horizontal_approach()) {
		return;
	}

	if (_mode == PrecLandMode::Opportunistic) {
		// could not see the target immediately, so just fall back to normal landing
		switch_to_state_fallback();
	}

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	float dist = get_distance_to_next_waypoint(pos_sp_triplet->current.lat, pos_sp_triplet->current.lon,
			_navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

	// check if we've reached the start point
	if (dist < _navigator->get_acceptance_radius()) {
		if (!_point_reached_time) {
			_point_reached_time = hrt_absolute_time();
		}

		// if we don't see the target after 1 second, search for it
		if (_param_pld_srch_tout.get() > 0) {

			if (hrt_absolute_time() - _point_reached_time > 2000000) {
				if (!switch_to_state_search()) {
					switch_to_state_fallback();
				}
			}

		} else {
			switch_to_state_fallback();
		}
	}
}

void
PrecLand::run_state_horizontal_approach()
{
	const uint64_t now = hrt_absolute_time();

	if (!_target_pose_valid) {
		PX4_WARN("%s, state: %i", LOST_TARGET_ERROR_MESSAGE, (int) _state);

		if (_mode == PrecLandMode::Required && switch_to_state_search()) {
			return;

		} else {
			switch_to_state_fallback();
		}

		return;
	}

	const matrix::Vector3f target_position = predicted_target(now);
	const float offset_z = update_vertical_offset(-_param_pld_hov_alt.get(), now);
	set_position_reference(target_position, offset_z);

	const vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();
	const float horizontal_distance = horizontal_error(target_position);
	const float desired_z = target_position(2) + offset_z;
	const bool settled = horizontal_distance < _param_pld_hset_rad.get()
			     && fabsf(desired_z - vehicle_local_position->z) < _param_pld_vacc_rad.get()
			     && fabsf(offset_z + _param_pld_hov_alt.get()) < 0.01f;

	if (settled) {
		if (_hover_settle_time == 0) {
			_hover_settle_time = now;

		} else if (hrt_elapsed_time(&_hover_settle_time) > _param_pld_hset_t.get() * SEC2USEC) {
			switch_to_state_descend_above_target();
		}

	} else {
		_hover_settle_time = 0;
	}
}

void
PrecLand::run_state_descend_above_target()
{
	const uint64_t now = hrt_absolute_time();

	if (target_age_s(now) > _param_pld_lost_tout.get()) {
		PX4_ERR("Landing target lost beyond hard timeout");
		switch_to_state_fallback();
		return;
	}

	if (!_target_pose_valid) {
		PX4_WARN("%s, state: %i", LOST_TARGET_ERROR_MESSAGE, (int) _state);

		if (_mode == PrecLandMode::Required && switch_to_state_search()) {
			return;

		} else {
			switch_to_state_fallback();
			return;
		}
	}

	const matrix::Vector3f target_position = predicted_target(now);
	const float horizontal_distance = horizontal_error(target_position);
	float target_offset = _current_offset_z;

	if (horizontal_distance < _param_pld_hacc_rad.get()) {
		target_offset = 0.f;

	} else if (horizontal_distance > _param_pld_reapp_rad.get()) {
		target_offset = -_param_pld_hov_alt.get();
	}

	const float offset_z = update_vertical_offset(target_offset, now);
	set_position_reference(target_position, offset_z);

	if (fabsf(offset_z) < _param_pld_fappr_alt.get()) {
		switch_to_state_final_approach();
	}
}

void
PrecLand::run_state_final_approach()
{
	const uint64_t now = hrt_absolute_time();

	if (target_age_s(now) > _param_pld_lost_tout.get()) {
		PX4_ERR("Landing target lost during final approach");
		switch_to_state_fallback();
		return;
	}

	// Losing the marker is expected close to the surface. Keep the landing
	// point moving with the last estimated target velocity until the hard
	// timeout above, instead of freezing it in the world frame.
	set_position_reference(predicted_target(now, true), 0.f, position_setpoint_s::SETPOINT_TYPE_LAND);
}

void
PrecLand::run_state_search()
{
	const uint64_t now = hrt_absolute_time();

	// check if we can see the target
	if (_target_pose_updated && _target_pose_valid) {
		if (!_target_acquired_time) {
			// Stop the search pattern as soon as the target is seen, but require
			// a stable acquisition before resuming the horizontal approach.
			_target_acquired_time = now;
		}
	}

	if (!_target_pose_valid) {
		_target_acquired_time = 0;
	}

	// stay at that height for a second to allow the vehicle to settle
	if (_target_acquired_time && (hrt_absolute_time() - _target_acquired_time) > 1000000) {
		// try to switch to horizontal approach
		if (switch_to_state_horizontal_approach()) {
			return;
		}
	}

	if (_target_pose_valid) {
		// During the one-second reacquisition settle time, already move the
		// search reference back above the live target instead of waiting at
		// search pattern setpoint.
		const float offset_z = update_vertical_offset(-_param_pld_hov_alt.get(), now);
		set_position_reference(predicted_target(now), offset_z);

	} else {
		if (update_search_reference(now)) {
			return;
		}
	}
}

void
PrecLand::run_state_fallback()
{
	// nothing to do, will land
}

bool
PrecLand::switch_to_state_start()
{
	if (check_state_conditions(PrecLandState::Start)) {
		position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		_navigator->set_position_setpoint_triplet_updated();
		_point_reached_time = 0;
		_hover_settle_time = 0;

		_state = PrecLandState::Start;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_horizontal_approach()
{
	if (check_state_conditions(PrecLandState::HorizontalApproach)) {
		print_state_switch_message("horizontal approach");
		_point_reached_time = 0;
		_hover_settle_time = 0;
		_last_offset_update = hrt_absolute_time();

		_state = PrecLandState::HorizontalApproach;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_descend_above_target()
{
	if (check_state_conditions(PrecLandState::DescendAboveTarget)) {
		print_state_switch_message("descend");
		_last_offset_update = hrt_absolute_time();
		_state = PrecLandState::DescendAboveTarget;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_final_approach()
{
	if (check_state_conditions(PrecLandState::FinalApproach)) {
		print_state_switch_message("final approach");
		_state = PrecLandState::FinalApproach;
		_state_start_time = hrt_absolute_time();
		set_position_reference(predicted_target(_state_start_time, true), 0.f,
				       position_setpoint_s::SETPOINT_TYPE_LAND);
		return true;
	}

	return false;
}

bool
PrecLand::switch_to_state_search()
{
	if (_search_cnt >= _param_pld_max_srch.get()) {
		return false;
	}

	const uint64_t now = hrt_absolute_time();
	const vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	if (_have_target_ever) {
		const matrix::Vector3f target_position = predicted_target(now);
		_search_center = matrix::Vector2f{target_position(0), target_position(1)};
		_search_altitude_z = target_position(2) - _param_pld_hov_alt.get();

	} else {
		_search_center = matrix::Vector2f{vehicle_local_position->x, vehicle_local_position->y};
		_search_altitude_z = -_param_pld_srch_alt.get();
	}

	const PrecLandSearchType search_type = static_cast<PrecLandSearchType>(_param_pld_srch_type.get());
	PX4_INFO("Starting %s landing-target search",
		 search_type == PrecLandSearchType::Spiral ? "spiral" : "simple");
	set_position_reference(matrix::Vector3f{_search_center(0), _search_center(1), _search_altitude_z}, 0.f);

	_target_acquired_time = 0;
	_search_pattern_start_time = 0;
	_search_return_start_time = 0;
	_search_phase = PrecLandSearchPhase::Climb;
	_last_offset_update = now;
	_search_cnt++;

	_state = PrecLandState::Search;
	_state_start_time = now;
	return true;
}

bool
PrecLand::switch_to_state_fallback()
{
	print_state_switch_message("fallback");
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
	pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
	pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LAND;
	_navigator->set_position_setpoint_triplet_updated();

	_state = PrecLandState::Fallback;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_fallback_at_search_center()
{
	print_state_switch_message("fallback at search center");
	set_position_reference(matrix::Vector3f{_search_center(0), _search_center(1), _search_altitude_z}, 0.f,
			       position_setpoint_s::SETPOINT_TYPE_LAND);

	_state = PrecLandState::Fallback;
	_state_start_time = hrt_absolute_time();
	return true;
}

bool
PrecLand::switch_to_state_done()
{
	_state = PrecLandState::Done;
	_state_start_time = hrt_absolute_time();
	return true;
}

void PrecLand::print_state_switch_message(const char *state_name)
{
	PX4_INFO("Precland: switching to %s", state_name);
}

void PrecLand::publish_status()
{
	uint8_t state = prec_land_status_s::STATE_START;

	switch (_state) {
	case PrecLandState::Start:              state = prec_land_status_s::STATE_START; break;

	case PrecLandState::HorizontalApproach: state = prec_land_status_s::STATE_HORIZONTAL_APPROACH; break;

	case PrecLandState::DescendAboveTarget: state = prec_land_status_s::STATE_DESCEND_ABOVE_TARGET; break;

	case PrecLandState::FinalApproach:      state = prec_land_status_s::STATE_FINAL_APPROACH; break;

	case PrecLandState::Search:             state = prec_land_status_s::STATE_SEARCH; break;

	case PrecLandState::Fallback:           state = prec_land_status_s::STATE_FALLBACK; break;

	case PrecLandState::Done:               state = prec_land_status_s::STATE_DONE; break;
	}

	prec_land_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.state = state;
	status.mode = (_mode == PrecLandMode::Required)
		      ? prec_land_status_s::MODE_REQUIRED
		      : prec_land_status_s::MODE_OPPORTUNISTIC;
	status.target_pose_valid = _target_pose_valid;
	status.search_count = _search_cnt;
	_precland_status_pub.publish(status);
}

bool PrecLand::check_state_conditions(PrecLandState state)
{
	switch (state) {
	case PrecLandState::Start:
		return true;

	case PrecLandState::HorizontalApproach:
		return _target_pose_valid && _target_pose.abs_pos_valid;

	case PrecLandState::DescendAboveTarget:
		return _target_pose_valid && _target_pose.abs_pos_valid;

	case PrecLandState::FinalApproach:
		return _have_target_ever && fabsf(_current_offset_z) < _param_pld_fappr_alt.get();

	case PrecLandState::Search:
		return true;

	case PrecLandState::Fallback:
		return true;

	default:
		return false;
	}
}

bool PrecLand::update_target_estimate()
{
	landing_target_pose_s measurement{};

	if (!_target_pose_sub.update(&measurement) || !measurement.abs_pos_valid) {
		return false;
	}

	const matrix::Vector3f measurement_position{measurement.x_abs, measurement.y_abs, measurement.z_abs};

	if (!measurement_position.isAllFinite() || measurement.timestamp == 0) {
		return false;
	}

	if (_have_target_ever && _target_pose.timestamp != 0) {
		if (measurement.timestamp <= _target_pose.timestamp) {
			return false;
		}

		const float dt = static_cast<float>(measurement.timestamp - _target_pose.timestamp) / SEC2USEC;

		if (dt > 0.f && dt < 1.f) {
			const matrix::Vector3f expected_position = predicted_target(measurement.timestamp);
			const float innovation = (measurement_position - expected_position).norm();
			const float max_plausible = _param_pld_out_marg.get() + _param_pld_tgt_vmax.get() * dt;

			if (innovation > max_plausible) {
				if (_consecutive_outlier_rejections < _param_pld_out_max.get()) {
					_consecutive_outlier_rejections++;
					PX4_WARN("Reject target outlier %.2fm (max %.2fm), %d/%d",
						 (double)innovation, (double)max_plausible,
						 _consecutive_outlier_rejections, _param_pld_out_max.get());
					return false;
				}

				// A sustained displacement is probably real. Accept it without
				// deriving velocity over the rejected interval.
				_target_velocity.zero();

			} else {
				const matrix::Vector3f previous_position{_target_pose.x_abs, _target_pose.y_abs, _target_pose.z_abs};
				const matrix::Vector3f raw_velocity = (measurement_position - previous_position) / dt;
				const float alpha = math::constrain(_param_pld_vel_alpha.get(), 0.f, 1.f);
				_target_velocity = raw_velocity * alpha + _target_velocity * (1.f - alpha);
			}
		}
	}

	_target_pose = measurement;
	_target_pose_valid = true;
	_have_target_ever = true;
	_consecutive_outlier_rejections = 0;
	update_target_yaw(measurement);
	return true;
}

void PrecLand::update_target_yaw(const landing_target_pose_s &measurement)
{
	if (_param_pld_yaw_en.get() == 0) {
		_target_yaw_valid = false;
		return;
	}

	// Temporary transport convention shared with aruco_detector_node.py:
	// rel_vel_valid stays false and vx_rel/vy_rel contain the unit heading
	// vector. A near-zero vector means that no target orientation was sent.
	const matrix::Vector2f heading_vector{measurement.vx_rel, measurement.vy_rel};

	if (!heading_vector.isAllFinite() || heading_vector.norm_squared() < 0.25f) {
		return;
	}

	const float measured_yaw = matrix::wrap_pi(atan2f(heading_vector(1), heading_vector(0))
				   + math::radians(_param_pld_yaw_off.get()));
	float dt = 0.05f;

	if (_last_yaw_update != 0 && measurement.timestamp > _last_yaw_update) {
		dt = math::constrain(static_cast<float>(measurement.timestamp - _last_yaw_update) / SEC2USEC, 0.f, 0.2f);
	}

	_last_yaw_update = measurement.timestamp;

	if (!_target_yaw_valid) {
		const float vehicle_heading = _navigator->get_local_position()->heading;
		_target_yaw = PX4_ISFINITE(vehicle_heading) ? vehicle_heading : measured_yaw;
		_target_yaw_valid = true;
	}

	const float alpha = math::constrain(_param_pld_yaw_alpha.get(), 0.f, 1.f);
	const float filtered_error = matrix::wrap_pi(measured_yaw - _target_yaw) * alpha;
	const float max_step = math::max(_param_pld_yaw_rate.get(), 0.f) * dt;
	_target_yaw = matrix::wrap_pi(_target_yaw + math::constrain(filtered_error, -max_step, max_step));
}

matrix::Vector3f PrecLand::predicted_target(uint64_t now, bool unclamped) const
{
	if (!_have_target_ever) {
		return matrix::Vector3f{};
	}

	float age = 0.f;

	if (now > _target_pose.timestamp) {
		age = static_cast<float>(now - _target_pose.timestamp) / SEC2USEC;
	}

	if (!unclamped) {
		age = math::constrain(age, 0.f, _param_pld_pred_t.get());
	}

	const matrix::Vector3f position{_target_pose.x_abs, _target_pose.y_abs, _target_pose.z_abs};
	return position + _target_velocity * age;
}

float PrecLand::target_age_s(uint64_t now) const
{
	if (!_have_target_ever || _target_pose.timestamp == 0) {
		return INFINITY;
	}

	return now > _target_pose.timestamp
	       ? static_cast<float>(now - _target_pose.timestamp) / SEC2USEC
	       : 0.f;
}

float PrecLand::update_vertical_offset(float target_offset, uint64_t now)
{
	float dt = 0.05f;

	if (_last_offset_update != 0 && now > _last_offset_update) {
		dt = math::constrain(static_cast<float>(now - _last_offset_update) / SEC2USEC, 0.f, 0.2f);
	}

	_last_offset_update = now;
	const float max_step = _param_pld_offs_rate.get() * dt;
	_current_offset_z += math::constrain(target_offset - _current_offset_z, -max_step, max_step);
	return _current_offset_z;
}

void PrecLand::set_position_reference(const matrix::Vector3f &target_position, float offset_z, uint8_t setpoint_type)
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	_map_ref.reproject(target_position(0), target_position(1),
			   pos_sp_triplet->current.lat, pos_sp_triplet->current.lon);

	// Local NED z is positive down, while the triplet altitude is AMSL and
	// positive up. offset_z is negative while holding above the target.
	pos_sp_triplet->current.alt = vehicle_local_position->ref_alt - (target_position(2) + offset_z);
	pos_sp_triplet->current.type = setpoint_type;
	pos_sp_triplet->current.valid = true;
	pos_sp_triplet->current.timestamp = hrt_absolute_time();

	if (_param_pld_yaw_en.get() != 0 && _target_yaw_valid) {
		pos_sp_triplet->current.yaw = _target_yaw;
	}

	_navigator->set_position_setpoint_triplet_updated();
}

float PrecLand::horizontal_error(const matrix::Vector3f &target_position) const
{
	const vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();
	const matrix::Vector2f error{target_position(0) - vehicle_local_position->x,
				     target_position(1) - vehicle_local_position->y};
	return error.norm();
}

bool PrecLand::update_search_reference(uint64_t now)
{
	matrix::Vector3f search_reference{_search_center(0), _search_center(1), _search_altitude_z};
	const PrecLandSearchType search_type = static_cast<PrecLandSearchType>(_param_pld_srch_type.get());
	const vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();
	const bool horizontally_centered = horizontal_error(search_reference) < _param_pld_hset_rad.get();
	const bool at_search_altitude = fabsf(vehicle_local_position->z - _search_altitude_z) < _param_pld_vacc_rad.get();

	const float watchdog_s = _param_pld_srch_wdog.get();

	if (_search_phase != PrecLandSearchPhase::Return && watchdog_s > 0.f
	    && now - _state_start_time > watchdog_s * SEC2USEC) {
		PX4_WARN("Search watchdog expired, returning to center");
		_search_phase = PrecLandSearchPhase::Return;
		_search_return_start_time = now;
	}

	if (_search_phase == PrecLandSearchPhase::Climb && horizontally_centered && at_search_altitude) {
		_search_phase = PrecLandSearchPhase::Pattern;
		_search_pattern_start_time = now;
		PX4_INFO("Landing-target search pattern started");
	}

	if (_search_phase == PrecLandSearchPhase::Pattern) {
		const float elapsed = static_cast<float>(now - _search_pattern_start_time) / SEC2USEC;

		if (search_type == PrecLandSearchType::Simple) {
			if (elapsed > _param_pld_srch_tout.get()) {
				PX4_WARN("Simple search completed without target");
				return switch_to_state_fallback_at_search_center();
			}

		} else {
			const int turns = math::max(_param_pld_srch_turns.get(), 1);
			const float angular_speed = math::radians(math::max(_param_pld_srch_omega.get(), 1.f));
			const float total_angle = 2.f * M_PI_F * static_cast<float>(turns);
			const float angle = math::min(angular_speed * elapsed, total_angle);
			const float radius = _param_pld_srch_rstep.get() * angle / (2.f * M_PI_F);

			search_reference(0) += radius * cosf(angle);
			search_reference(1) += radius * sinf(angle);

			if (angle >= total_angle) {
				PX4_INFO("Spiral search completed, returning to center");
				_search_phase = PrecLandSearchPhase::Return;
				_search_return_start_time = now;
				search_reference = matrix::Vector3f{_search_center(0), _search_center(1), _search_altitude_z};
			}
		}
	}

	if (_search_phase == PrecLandSearchPhase::Return) {
		search_reference = matrix::Vector3f{_search_center(0), _search_center(1), _search_altitude_z};

		if (horizontally_centered) {
			PX4_INFO("Search center reached, landing");
			return switch_to_state_fallback_at_search_center();
		}

		if (_search_return_start_time != 0
		    && now - _search_return_start_time > _param_pld_srch_tout.get() * SEC2USEC) {
			PX4_WARN("Return to search center timed out");
			return switch_to_state_fallback();
		}
	}

	set_position_reference(search_reference, 0.f);
	return false;
}
