/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#pragma once

#include <gz/sim/Util.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/sim/System.hh>
#include "gz/sim/components/Pose.hh"
#include <gz/sim/components/Model.hh>
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include <gz/sim/EntityComponentManager.hh>

#include <gz/common/Timer.hh>

#include <gz/plugin/Register.hh>

#include <gz/msgs/Utility.hh>
#include <gz/msgs/twist.pb.h>

#include <gz/math.hh>
#include <gz/math/Rand.hh>
#include <gz/math/Pose3.hh>

#include <cmath>

using namespace std::chrono_literals;

namespace custom
{
class MovingPlatformController:
	public gz::sim::System,
	public gz::sim::ISystemPreUpdate,
	public gz::sim::ISystemConfigure
{
public:
	void PreUpdate(const gz::sim::UpdateInfo &_info,
		       gz::sim::EntityComponentManager &_ecm) final;

	void Configure(const gz::sim::Entity &entity,
		       const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm,
		       gz::sim::EventManager &eventMgr) override;

private:

	void getPlatformState(const gz::sim::EntityComponentManager &ecm);
	void updateNoise(const double dt);
	void updateWrenchCommand(const gz::math::Vector3d &velocity_setpoint,
				 const gz::math::Quaterniond &orientation_setpoint,
				 const bool keep_stationary);
	void sendWrenchCommand(gz::sim::EntityComponentManager &ecm);
	double readEnvVar(const char *env_var_name, double default_value);
	void getVehicleModelName();

	gz::sim::Entity _entity;
	gz::sim::Model _model{gz::sim::kNullEntity};
	gz::sim::Entity _link_entity;
	gz::sim::Link _link;

	gz::sim::Entity _world_entity;
	gz::sim::World _world;

	// Low-pass filtered white noise for driving boat motion.
	gz::math::Vector3d _noise_lowpass_force{0., 0., 0.};
	gz::math::Vector3d _noise_lowpass_torque{0., 0., 0.};

	// Platform linear & angular velocity.
	gz::math::Vector3d _force{0., 0., 0.};
	gz::math::Vector3d _torque{0., 0., 0.};

	// Platform position & orientation for feedback.
	gz::math::Vector3d _platform_position{0., 0., 0.};
	gz::math::Quaterniond _platform_orientation{1., 0., 0., 0.};
	gz::math::Vector3d _platform_velocity{0., 0., 0.};
	gz::math::Vector3d _platform_angular_velocity{0., 0., 0.};

	// Platform velocity setpoint [m/s].
	gz::math::Vector3d _velocity_sp{1., 0., 0.};
	// Orientation setpoint.
	gz::math::Quaterniond _orientation_sp{1., 0., 0., 0.};
	// Height setpoint [m]
	double _platform_height_setpoint{2.};
	// Sinusoidal heave (vertical bob) added on top of the height setpoint,
	// to emulate a boat/ship riding waves. Amplitude 0 (default) = disabled,
	// matching prior behaviour when the env vars below are unset.
	double _heave_amplitude{0.};   // [m]
	double _heave_period{6.};      // [s]
	// Current simulation time, refreshed every PreUpdate, used to phase the heave.
	double _sim_time_sec{0.};
	// Scales the random-noise wrench (waves/road noise jitter) applied every
	// step. 0 (default) disables it; 1.0 matches the original always-on jitter.
	double _noise_amplitude{0.};

	// How long (sim seconds) to keep the platform stationary after the
	// vehicle has spawned, so there's time to arm/take off with a stable
	// GPS fix before the platform starts moving. 0 (default) = original
	// behaviour: starts moving as soon as the vehicle spawns.
	double _start_delay_sec{0.};
	// Sim time at which the vehicle was first seen spawned; -1 = not yet.
	double _vehicle_spawn_sim_time{-1.};

	// If > 0 (PX4_GZ_PLATFORM_TRAVEL_DISTANCE), the platform stops for good
	// once it has moved this many horizontal metres from where it was when
	// it started moving. 0 (default) = original behaviour: moves forever.
	double _travel_distance{0.};
	gz::math::Vector3d _motion_start_position{0., 0., 0.};
	bool _motion_started{false};
	bool _travel_limit_reached{false};

	double _gravity{-9.8};
	double _platform_mass{10000.};
	gz::math::Vector3d _platform_diag_moments;

	gz::common::Timer _startup_timer;
	std::string _vehicle_model_name;
	bool _wait_for_vehicle_spawned;
};
} // end namespace custom
