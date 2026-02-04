/****************************************************************************
 *
 *   Copyright (c) 2025
 *
 *   Permesso d'uso conforme alla licenza PX4 (BSD-3-Clause).
 *
 ****************************************************************************/

#include "camera_input_rc.h"
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
#include <uORB/topics/vehicle_command.h>

CameraInputRC *CameraInputRC::instantiate(int argc, char *argv[])
{
    return new CameraInputRC();
}

int CameraInputRC::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("camera_input_rc",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  1800,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        PX4_ERR("task start failed");
        return -errno;
    }

    return PX4_OK;
}

int CameraInputRC::custom_command(int argc, char *argv[])
{
    // Per ora nessun comando custom: mostriamo l'uso
    return print_usage("Unknown command");
}

int CameraInputRC::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Listens to an RC AUX channel and triggers the camera via vehicle_command.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("camera_input_rc", "module");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}



void CameraInputRC::run()
{
	while (!should_exit()) {
		manual_control_setpoint_s msp{};

		if (_manual_sub.update(&msp)) {
			// Leggi quale AUX è mappato nei parametri
			int aux = _param_cam_man_aux.get();
			float threshold = _param_cam_man_thr.get();

			float val = 0.f;

			switch (aux) {
			case 1: val = msp.aux1; break;
			case 2: val = msp.aux2; break;
			case 3: val = msp.aux3; break;
			case 4: val = msp.aux4; break;
			case 5: val = msp.aux5; break;
			case 6: val = msp.aux6; break;
			default: break;
			}


			// Stato attuale ON/OFF rispetto alla soglia
			bool state = (val > threshold);

			// Edge detection: scatta solo quando passi da OFF -> ON
			if (state && !_last_state) {
				vehicle_command_s vcmd{};
				vcmd.timestamp = hrt_absolute_time();
				vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_DIGICAM_CONTROL;

				// param5 = shutter control
				vcmd.param5 = 1.0f;

				vcmd.target_system = 1;
				vcmd.target_component = 1;

				_vcmd_pub.publish(vcmd);

				PX4_INFO("Camera trigger via RC (AUX%i)", aux);
			}

			_last_state = state;
		}

		// Frequenza di loop ~50 Hz
		px4_usleep(20000);
	}
}

extern "C" __EXPORT int camera_input_rc_main(int argc, char *argv[])
{
	return CameraInputRC::main(argc, argv);
}
