/***************************************************************************
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
 * @file precland.h
 *
 * Helper class to do precision landing with a landing target
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#pragma once

#include <matrix/math.hpp>
#include <lib/geo/geo.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/prec_land_status.h>

#include "navigator_mode.h"
#include "mission_block.h"

enum class PrecLandState {
	Start, // Starting state
	HorizontalApproach, // Positioning at PLD_HOV_ALT above the landing target
	DescendAboveTarget, // Position-mode descent ratchet with horizontal re-approach
	FinalApproach, // Final landing approach, even without landing target
	Search, // Search for landing target
	Fallback, // Fallback landing method
	Done // Done landing
};

enum class PrecLandMode {
	Opportunistic = 1, // only do precision landing if landing target visible at the beginning
	Required = 2 // try to find landing target if not visible at the beginning
};

enum class PrecLandSearchType {
	Simple = 0, // climb above the search center and hold
	Spiral = 1 // climb, then expand a horizontal spiral around the search center
};

enum class PrecLandSearchPhase {
	Climb,
	Pattern,
	Return
};

class PrecLand : public MissionBlock, public ModuleParams
{
public:
	PrecLand(Navigator *navigator);
	~PrecLand() override = default;

	void on_activation() override;
	void on_active() override;
	void on_inactivation() override;

	void set_mode(PrecLandMode mode) { _mode = mode; };

	PrecLandMode get_mode() { return _mode; };

	bool is_activated() { return _is_activated; };

private:

	void updateParams() override;

	// run the control loop for each state
	void run_state_start();
	void run_state_horizontal_approach();
	void run_state_descend_above_target();
	void run_state_final_approach();
	void run_state_search();
	void run_state_fallback();

	// attempt to switch to a different state. Returns true if state change was successful, false otherwise
	bool switch_to_state_start();
	bool switch_to_state_horizontal_approach();
	bool switch_to_state_descend_above_target();
	bool switch_to_state_final_approach();
	bool switch_to_state_search();
	bool switch_to_state_fallback();
	bool switch_to_state_fallback_at_search_center();
	bool switch_to_state_done();

	void print_state_switch_message(const char *state_name);
	bool update_target_estimate();
	void update_target_yaw(const landing_target_pose_s &measurement);
	matrix::Vector3f predicted_target(uint64_t now, bool unclamped = false) const;
	float target_age_s(uint64_t now) const;
	float update_vertical_offset(float target_offset, uint64_t now);
	void set_position_reference(const matrix::Vector3f &target_position, float offset_z,
				    uint8_t setpoint_type = position_setpoint_s::SETPOINT_TYPE_POSITION);
	float horizontal_error(const matrix::Vector3f &target_position) const;
	bool update_search_reference(uint64_t now);

	// check if a given state could be changed into. Return true if possible to transition to state, false otherwise
	bool check_state_conditions(PrecLandState state);

	// publish _state/_mode/etc as prec_land_status, so the phase precland is
	// in shows up in flight logs (and can be watched live) -- nothing else
	// exposes it outside of this class.
	void publish_status();

	landing_target_pose_s _target_pose{}; /**< precision landing target position */

	uORB::Subscription _target_pose_sub{ORB_ID(landing_target_pose)};
	uORB::Publication<prec_land_status_s> _precland_status_pub{ORB_ID(prec_land_status)};
	bool _target_pose_valid{false}; /**< whether we have received a landing target position message */
	bool _target_pose_updated{false}; /**< wether the landing target position message is updated */
	bool _have_target_ever{false};
	matrix::Vector3f _target_velocity{};
	int _consecutive_outlier_rejections{0};
	bool _target_yaw_valid{false};
	float _target_yaw{0.f};
	uint64_t _last_yaw_update{0};

	MapProjection _map_ref{}; /**< class for local/global projections */

	uint64_t _state_start_time{0}; /**< time when we entered current state */
	uint64_t _target_acquired_time{0}; /**< time when we first saw the landing target during search */
	uint64_t _point_reached_time{0}; /**< time when we reached a setpoint */
	uint64_t _hover_settle_time{0}; /**< time since the hover reference has continuously been settled */
	uint64_t _last_offset_update{0}; /**< time used to rate-limit the vertical reference offset */
	uint64_t _search_pattern_start_time{0}; /**< time the vehicle reached search altitude and started the pattern */
	uint64_t _search_return_start_time{0}; /**< time the vehicle started returning to the search center */

	int _search_cnt{0}; /**< counter of how many times we had to search for the landing target */
	float _current_offset_z{0.0f}; /**< NED vertical offset from target (negative means above it) */
	matrix::Vector2f _search_center{}; /**< local NED center of the active search pattern */
	float _search_altitude_z{0.0f}; /**< local NED altitude used by the active search pattern */
	PrecLandSearchPhase _search_phase{PrecLandSearchPhase::Climb};

	PrecLandState _state{PrecLandState::Start};

	PrecLandMode _mode{PrecLandMode::Opportunistic};

	bool _is_activated {false}; /**< indicates if precland is activated */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::PLD_BTOUT>) _param_pld_btout,
		(ParamFloat<px4::params::PLD_HACC_RAD>) _param_pld_hacc_rad,
		(ParamFloat<px4::params::PLD_FAPPR_ALT>) _param_pld_fappr_alt,
		(ParamFloat<px4::params::PLD_HOV_ALT>) _param_pld_hov_alt,
		(ParamFloat<px4::params::PLD_HSET_RAD>) _param_pld_hset_rad,
		(ParamFloat<px4::params::PLD_VACC_RAD>) _param_pld_vacc_rad,
		(ParamFloat<px4::params::PLD_HSET_T>) _param_pld_hset_t,
		(ParamFloat<px4::params::PLD_REAPP_RAD>) _param_pld_reapp_rad,
		(ParamFloat<px4::params::PLD_OFFS_RATE>) _param_pld_offs_rate,
		(ParamFloat<px4::params::PLD_LOST_TOUT>) _param_pld_lost_tout,
		(ParamFloat<px4::params::PLD_VEL_ALPHA>) _param_pld_vel_alpha,
		(ParamFloat<px4::params::PLD_PRED_T>) _param_pld_pred_t,
		(ParamFloat<px4::params::PLD_TGT_VMAX>) _param_pld_tgt_vmax,
		(ParamFloat<px4::params::PLD_OUT_MARG>) _param_pld_out_marg,
		(ParamInt<px4::params::PLD_OUT_MAX>) _param_pld_out_max,
		(ParamFloat<px4::params::PLD_SRCH_ALT>) _param_pld_srch_alt,
		(ParamFloat<px4::params::PLD_SRCH_TOUT>) _param_pld_srch_tout,
		(ParamInt<px4::params::PLD_MAX_SRCH>) _param_pld_max_srch,
		(ParamInt<px4::params::PLD_SRCH_TYPE>) _param_pld_srch_type,
		(ParamInt<px4::params::PLD_SRCH_TURNS>) _param_pld_srch_turns,
		(ParamFloat<px4::params::PLD_SRCH_RSTEP>) _param_pld_srch_rstep,
		(ParamFloat<px4::params::PLD_SRCH_OMEGA>) _param_pld_srch_omega,
		(ParamFloat<px4::params::PLD_SRCH_WDOG>) _param_pld_srch_wdog,
		(ParamInt<px4::params::PLD_YAW_EN>) _param_pld_yaw_en,
		(ParamFloat<px4::params::PLD_YAW_RATE>) _param_pld_yaw_rate,
		(ParamFloat<px4::params::PLD_YAW_ALPHA>) _param_pld_yaw_alpha,
		(ParamFloat<px4::params::PLD_YAW_OFF>) _param_pld_yaw_off
	)

};
