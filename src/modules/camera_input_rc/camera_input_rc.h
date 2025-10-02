/****************************************************************************
 *
 *   Copyright (c) 2025
 *
 *   Permesso d'uso conforme alla licenza PX4 (BSD-3-Clause).
 *
 ****************************************************************************/

#pragma once

#include <px4_platform_common/module.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>

#include <px4_platform_common/module_params.h>

class CameraInputRC : public ModuleBase<CameraInputRC>, public ModuleParams
{
public:
    CameraInputRC() : ModuleParams(nullptr) {}
    ~CameraInputRC() override = default;

    void run() override;
	static CameraInputRC *instantiate(int argc, char *argv[]);
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);



private:
    // Qui definisci i parametri
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::CAM_MAN_AUX>) _param_cam_man_aux,
        (ParamFloat<px4::params::CAM_MAN_THR>) _param_cam_man_thr
    )

    bool _last_state{false};
    uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};
    uORB::Publication<vehicle_command_s> _vcmd_pub{ORB_ID(vehicle_command)};
};