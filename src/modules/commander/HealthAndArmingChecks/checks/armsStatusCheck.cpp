#include "armsStatusCheck.hpp"
#include <cstdio>    // per snprintf
#include <cstring>   // per strcat

void ArmsStatusCheck::checkAndReport(const Context &context, Report &reporter)
{
	if (_param_arms_closed_check.get()) {

		arms_status_s arms{};

		if (_arms_status_sub.copy(&arms)) {

			char open_arms[32] = "";
			bool any_open = false;

			if (arms.hall1 == 0) { strcat(open_arms, "1 "); any_open = true; }
			if (arms.hall2 == 0) { strcat(open_arms, "2 "); any_open = true; }
			if (arms.hall3 == 0) { strcat(open_arms, "3 "); any_open = true; }
			if (arms.hall4 == 0) { strcat(open_arms, "4 "); any_open = true; }

			if (any_open) {
				char msg[64];
				snprintf(msg, sizeof(msg), "Bracci non chiusi: %s", open_arms);

				/* EVENT
				 * @description
				 * Maestro chiuda i bracci per favore !!!!!!!!!!!!!!!!!1
				 */
				reporter.armingCheckFailure(
					NavModes::All,
					health_component_t::system,
					events::ID("check_arms_not_closed"),
					events::Log::Error,
					"Bracci non chiusi");

				if (reporter.mavlink_log_pub()) {
					mavlink_log_critical(reporter.mavlink_log_pub(),
					                     "Preflight Fail: %s", msg);
				}
			}
		}
	}
}
