#include "../Common.hpp"
#include <uORB/Subscription.hpp>
#include <uORB/topics/arms_status.h>
#include <lib/events/events.h>  // per eventi e logging

class ArmsStatusCheck : public HealthAndArmingCheckBase
{
public:
	ArmsStatusCheck() = default;
	~ArmsStatusCheck() override = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	uORB::Subscription _arms_status_sub{ORB_ID(arms_status)};

    DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamInt<px4::params::CLOSED_ARMS_CHK>) _param_arms_closed_check
				       )
};