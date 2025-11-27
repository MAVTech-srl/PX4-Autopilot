/****************************************************************************
 *
 *   Copyright (c) 2018-2023 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManualAltitude.cpp
 */

#include "FlightTaskManualAltitude.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <geo/geo.h>

using namespace matrix;

bool FlightTaskManualAltitude::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();

	_sticks.checkAndUpdateStickInputs();

	if (_sticks_data_required) {
		ret = ret && _sticks.isAvailable();
	}

	// in addition to manual require valid position and velocity in D-direction and valid yaw
	return ret && PX4_ISFINITE(_position(2)) && PX4_ISFINITE(_velocity(2)) && PX4_ISFINITE(_yaw);
}

bool FlightTaskManualAltitude::activate(const trajectory_setpoint_s &last_setpoint)
{
	bool ret = FlightTask::activate(last_setpoint);
	_yaw_setpoint = NAN;
	_yawspeed_setpoint = 0.f;
	_acceleration_setpoint = Vector3f(0.f, 0.f, NAN); // altitude is controlled from position/velocity
	_position_setpoint(2) = _position(2);
	_velocity_setpoint(2) = 0.f;
	_stick_yaw.reset(_yaw, _unaided_yaw);
	_setDefaultConstraints();

	_updateConstraintsFromEstimator();

	return ret;
}

void FlightTaskManualAltitude::_updateConstraintsFromEstimator()
{
	if (PX4_ISFINITE(_sub_vehicle_local_position.get().hagl_min)) {
		_min_distance_to_ground = _sub_vehicle_local_position.get().hagl_min;

	} else {
		_min_distance_to_ground = -INFINITY;
	}

	if (!PX4_ISFINITE(_max_distance_to_ground) && PX4_ISFINITE(_sub_vehicle_local_position.get().hagl_max_z)) {
		_max_distance_to_ground = _sub_vehicle_local_position.get().hagl_max_z;
	}
}

void FlightTaskManualAltitude::_scaleSticks()
{
	// Use sticks input with deadzone and exponential curve for vertical velocity
	const float vel_max_up   = fminf(_param_mpc_z_vel_max_up.get(), _velocity_constraint_up);
	const float vel_max_down = fminf(_param_mpc_z_vel_max_dn.get(), _velocity_constraint_down);

	const float stick_input = _sticks.getPositionExpo()(2); // positivo = giù (discesa), negativo = su (salita)

	float vel_target = 0.f;

	// di default nessun freno CP_Z
	_cp_z_brake_active = false;

	if (stick_input > 0.f) {
		// discesa
		vel_target = vel_max_down * stick_input;

		// --- Collision Prevention Z ---
		if (_param_cp_dist_z.get() > 0.f &&
		    _sub_vehicle_local_position.get().dist_bottom_valid &&
		    PX4_ISFINITE(_dist_to_bottom)) {

			const float stop_distance = _dist_to_bottom - _param_cp_dist_z.get();

			if (stop_distance <= 0.12f) {
				// troppo vicino all’ostacolo: blocca la discesa
				vel_target = 0.f;
				_cp_z_brake_active = true;

			} else {
				// riduco velocità in base allo spazio disponibile
				const float safe_vel_down = math::trajectory::computeMaxSpeedFromDistance(
					_param_mpc_jerk_max.get(),
					_param_mpc_acc_down_max.get(),
					stop_distance,
					0.f);

				vel_target = math::min(vel_target, safe_vel_down);
			}
		}

	} else {
		// salita
		vel_target = vel_max_up * stick_input;
	}

	_velocity_setpoint(2) = vel_target;
}

void FlightTaskManualAltitude::_updateAltitudeLock()
{
	// Depending on stick inputs and velocity, position is locked.
	// If not locked, altitude setpoint is set to NAN.
	if (_cp_z_brake_active) {
		// Se non avevamo ancora un setpoint di posizione, bloccalo ora
		if (!PX4_ISFINITE(_position_setpoint(2))) {
			_position_setpoint(2) = _position(2);
		}

		// Non comandiamo più discesa
		_velocity_setpoint(2) = 0.f;

		// Continua a rispettare il vincolo di massima quota
		_respectMaxAltitude();

		// Non far eseguire la logica sotto (che potrebbe mettere NAN)
		return;
	}


	// Check if user wants to break
	const bool apply_brake = fabsf(_sticks.getPositionExpo()(2)) <= FLT_EPSILON;

	// Check if vehicle has stopped
	const bool stopped = (_param_mpc_hold_max_z.get() < FLT_EPSILON
			      || fabsf(_velocity(2)) < _param_mpc_hold_max_z.get());

	// Manage transition between use of distance to ground and distance to local origin
	// when terrain hold behaviour has been selected.
	if (_param_mpc_alt_mode.get() == 2) {
		// Use horizontal speed as a transition criteria
		float spd_xy = Vector2f(_velocity).length();

		// Use presence of horizontal stick inputs as a transition criteria
		float stick_xy = Vector2f(_sticks.getPitchRollExpo()).length();
		bool stick_input = stick_xy > 0.001f;

		if (_terrain_hold) {
			bool too_fast = spd_xy > _param_mpc_hold_max_xy.get();

			if (stick_input || too_fast || !PX4_ISFINITE(_dist_to_bottom)) {
				// Stop using distance to ground
				_terrain_hold = false;

				// Adjust the setpoint to maintain the same height error to reduce control transients
				if (PX4_ISFINITE(_dist_to_ground_lock) && PX4_ISFINITE(_dist_to_bottom)) {
					_position_setpoint(2) = _position(2) - (_dist_to_ground_lock - _dist_to_bottom);

				} else {
					_position_setpoint(2) = _position(2);
					_dist_to_ground_lock = NAN;
				}
			}

		} else {
			bool not_moving = spd_xy < 0.5f * _param_mpc_hold_max_xy.get() && stopped;

			if (!stick_input && not_moving && PX4_ISFINITE(_dist_to_bottom)) {
				// Start using distance to ground
				_terrain_hold = true;

				// Adjust the setpoint to maintain the same height error to reduce control transients
				if (PX4_ISFINITE(_position_setpoint(2))) {
					_dist_to_ground_lock = _dist_to_bottom - (_position_setpoint(2) - _position(2));
				}
			}
		}
	}

	if ((_param_mpc_alt_mode.get() == 1 || _terrain_hold) && PX4_ISFINITE(_dist_to_bottom)) {
		// terrain following
		_terrainFollowing(apply_brake, stopped);

	} else {
		// normal mode where height is dependent on local frame

		if (apply_brake && stopped && !PX4_ISFINITE(_position_setpoint(2))) {
			// lock position
			_position_setpoint(2) = _position(2);

			// Ensure that minimum altitude is respected if
			// there is a distance sensor and distance to bottom is below minimum.
			if (PX4_ISFINITE(_dist_to_bottom) && _dist_to_bottom < _min_distance_to_ground) {
				_terrainFollowing(apply_brake, stopped);

			} else {
				_dist_to_ground_lock = NAN;
			}

		} else if (PX4_ISFINITE(_position_setpoint(2)) && apply_brake) {
			// Position is locked but check if a reset event has happened.
			// We will shift the setpoints.
			if (_sub_vehicle_local_position.get().z_reset_counter != _reset_counter) {
				_position_setpoint(2) = _position(2);
				_reset_counter = _sub_vehicle_local_position.get().z_reset_counter;
			}

		} else  {
			// user demands velocity change
			_position_setpoint(2) = NAN;
			// ensure that maximum altitude is respected
		}
	}

	_respectMaxAltitude();
}



void FlightTaskManualAltitude::_respectMinAltitude()
{
	// Height above ground needs to be limited (flow / range-finder)
	if (PX4_ISFINITE(_dist_to_bottom) && (_dist_to_bottom < _min_distance_to_ground)) {
		// increase altitude to minimum flow distance
		_position_setpoint(2) = _position(2) - (_min_distance_to_ground - _dist_to_bottom);
	}
}

void FlightTaskManualAltitude::_terrainFollowing(bool apply_brake, bool stopped)
{
	if (apply_brake && stopped && !PX4_ISFINITE(_dist_to_ground_lock)) {
		// User wants to break and vehicle reached zero velocity. Lock height to ground.

		// lock position
		_position_setpoint(2) = _position(2);
		// ensure that minimum altitude is respected
		_respectMinAltitude();
		// lock distance to ground but adjust first for minimum altitude
		_dist_to_ground_lock = _dist_to_bottom - (_position_setpoint(2) - _position(2));

	} else if (apply_brake && PX4_ISFINITE(_dist_to_ground_lock)) {
		// vehicle needs to follow terrain

		// difference between the current distance to ground and the desired distance to ground
		const float delta_distance_to_ground = _dist_to_ground_lock - _dist_to_bottom;
		// adjust position setpoint for the delta (note: NED frame)
		_position_setpoint(2) = _position(2) - delta_distance_to_ground;

	} else {
		// user demands velocity change in D-direction
		_dist_to_ground_lock = _position_setpoint(2) = NAN;
	}
}

void FlightTaskManualAltitude::_respectMaxAltitude()
{
	if (PX4_ISFINITE(_dist_to_bottom)) {

		float vel_constrained = _param_mpc_z_p.get() * (_max_distance_to_ground - _dist_to_bottom);

		if (PX4_ISFINITE(_max_distance_to_ground)) {
			_constraints.speed_up = math::constrain(vel_constrained, -_param_mpc_z_vel_max_dn.get(), _param_mpc_z_vel_max_up.get());

		} else {
			_constraints.speed_up = _param_mpc_z_vel_max_up.get();
		}

		if (_dist_to_bottom > _max_distance_to_ground && !(_sticks.getThrottleZeroCenteredExpo() < FLT_EPSILON)) {
			_velocity_setpoint(2) = math::constrain(-vel_constrained, 0.f, _param_mpc_z_vel_max_dn.get());
		}

		_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
	}
}

void FlightTaskManualAltitude::_respectGroundSlowdown()
{
	// Interpolate descent rate between the altitudes MPC_LAND_ALT1 and MPC_LAND_ALT2
	if (PX4_ISFINITE(_dist_to_ground)) {
		const float limit_down = math::interpolate(_dist_to_ground,
					 _param_mpc_land_alt2.get(), _param_mpc_land_alt1.get(),
					 _param_mpc_land_speed.get(), _constraints.speed_down);
		const float limit_up = math::interpolate(_dist_to_ground,
				       _param_mpc_land_alt2.get(), _param_mpc_land_alt1.get(),
				       _param_mpc_tko_speed.get(), _constraints.speed_up);
		_velocity_setpoint(2) = math::constrain(_velocity_setpoint(2), -limit_up, limit_down);
	}
}

void FlightTaskManualAltitude::_ekfResetHandlerHeading(float delta_psi)
{
	// Only reset the yaw setpoint when the heading is locked
	if (PX4_ISFINITE(_yaw_setpoint)) {
		_yaw_setpoint = wrap_pi(_yaw_setpoint + delta_psi);
	}

	_stick_yaw.ekfResetHandler(delta_psi);
}

void FlightTaskManualAltitude::_ekfResetHandlerHagl(float delta_hagl)
{
	_dist_to_ground_lock = NAN;
}

void FlightTaskManualAltitude::_updateSetpoints()
{
	_stick_yaw.generateYawSetpoint(_yawspeed_setpoint, _yaw_setpoint, _sticks.getYawExpo(), _yaw, _deltatime, _unaided_yaw);
	_acceleration_setpoint.xy() = _stick_tilt_xy.generateAccelerationSetpoints(_sticks.getPitchRoll(), _deltatime, _yaw,
				      _yaw_setpoint);
	_updateAltitudeLock();
	_respectGroundSlowdown();
}

bool FlightTaskManualAltitude::_checkTakeoff()
{
	// stick is deflected above 65% throttle (throttle stick is in the range [-1,1])
	return _sticks.getPosition()(2) < -0.3f;
}

bool FlightTaskManualAltitude::update()
{
	bool ret = FlightTask::update();
	_updateConstraintsFromEstimator();
	_scaleSticks();
	_updateSetpoints();
	_constraints.want_takeoff = _checkTakeoff();
	_max_distance_to_ground = INFINITY;

	_checkForcedLand();

	return ret;
}


void FlightTaskManualAltitude::_checkForcedLand()
{
	using namespace time_literals;

	static hrt_abstime stick_down_start = 0;
	static hrt_abstime blocked_start = 0;
	static hrt_abstime arming_time = 0;

	// --- Stato veicolo ---
	vehicle_status_s vstatus{};
	_vehicle_status_sub.copy(&vstatus);   // SEMPRE, senza if!


	// Registra il tempo di arming
	if (vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED && arming_time == 0) {
		arming_time = hrt_absolute_time();
	}

	if (vstatus.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		arming_time = 0;
		stick_down_start = 0;
		blocked_start = 0;
		_forced_land_triggered = false;   // meglio: reset, non blocco
		return;
	}


	// Evita trigger nei primi 3 s dopo l'arming (arming + spool-up)
	if (hrt_elapsed_time(&arming_time) < 3_s) {
		return;
	}

	// Se già in LAND → non retriggerare
	if (_forced_land_triggered) {
    	// abbiamo già mandato un LAND, non fare altro
		return;
	}
	

	// --- Stick input ---
	const float stick_input = _sticks.getPositionExpo()(2); // positivo = giù
	const bool stick_down = (stick_input > 0.8f);

	// Reset se lo stick non è più tutto giù
	if (!stick_down) {
		stick_down_start = 0;
		blocked_start = 0;
		_forced_land_triggered = false;
		return;
	}

	
	// --- Condizione bloccato: CP_Z sta frenando ---
	const bool blocked = _cp_z_brake_active && (fabsf(_velocity(2)) < 0.3f);


	// Timer blocco CP_Z
	if (blocked) {
		if (blocked_start == 0) {
			blocked_start = hrt_absolute_time();
		}
	} else {
		blocked_start = 0;
	}

	// Timer stick giù
	if (stick_down && stick_down_start == 0) {
		stick_down_start = hrt_absolute_time();
	}

	// --- Condizione per trigger LAND ---
	const float hold_time_s = _param_cp_stktime_z.get();

	if (stick_down && blocked && !_forced_land_triggered &&
	    hrt_elapsed_time(&blocked_start) > 500_ms &&
	    hrt_elapsed_time(&stick_down_start) > (hold_time_s * 1_s)) {

		vehicle_command_s cmd{};
		cmd.timestamp = hrt_absolute_time();
		cmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
		cmd.target_system = 1;
		cmd.target_component = 1;
		cmd.source_system = 1;
		cmd.source_component = 1;
		cmd.from_external = false;

		_vehicle_command_pub.publish(cmd);

		PX4_INFO("FORCED LAND triggered (dist=%.2f vel=%.2f)",
			 (double)_dist_to_bottom,
			 (double)_velocity(2));

		_forced_land_triggered = true;
	}

	
}
