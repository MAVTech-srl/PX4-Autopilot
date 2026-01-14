#include "armsStatusCheck.hpp"
#include <cstdio>
#include <cstdint>

void ArmsStatusCheck::checkAndReport(const Context &context, Report &reporter)
{
	if (!_param_arms_closed_check.get()) {
		return;
	}

	arms_status_s arms{};

	if (!_arms_status_sub.copy(&arms)) {
		return;
	}

	char open_arms[32];
	open_arms[0] = '\0';
	size_t used = 0;

	auto append_arm = [&](int arm_idx) {
		if (used < sizeof(open_arms)) {
			used += snprintf(open_arms + used,
			                 sizeof(open_arms) - used,
			                 "%d ",
			                 arm_idx);
		}
	};

	bool any_open = false;

	// Semantics: hallX == 0  -> arm NOT closed
	if (arms.hall1 == 0) { append_arm(1); any_open = true; }
	if (arms.hall2 == 0) { append_arm(2); any_open = true; }
	if (arms.hall3 == 0) { append_arm(3); any_open = true; }
	if (arms.hall4 == 0) { append_arm(4); any_open = true; }

	if (!any_open) {
		return;
	}

	// Remove trailing space
	if (used > 0 && open_arms[used - 1] == ' ') {
		open_arms[used - 1] = '\0';
	}

	/* EVENT
	 * @description
	 * Arms not closed
	 */
	reporter.armingCheckFailure(
		NavModes::All,
		health_component_t::system,
		events::ID("check_arms_not_closed"),
		events::Log::Error,
		"Arms not closed");

	// Detailed info (allowed to be dynamic)
	if (reporter.mavlink_log_pub()) {
		mavlink_log_critical(
			reporter.mavlink_log_pub(),
			"Preflight Fail: Arms not closed: %s",
			open_arms);
	}
}
