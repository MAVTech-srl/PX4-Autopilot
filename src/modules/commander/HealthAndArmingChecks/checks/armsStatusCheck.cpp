#include "armsStatusCheck.hpp"

void ArmsStatusCheck::checkAndReport(const Context &context, Report &reporter)
{
    PX4_INFO("ARMS_CLOSED_CHK = %d", (int)_param_arms_closed_check.get());

    if (_param_arms_closed_check.get()){
        arms_status_s arms{};

        // Se c’è un messaggio valido da uORB
        if (_arms_status_sub.copy(&arms)) {

            if (arms.button1 == 0) {
                /* EVENT
                *@description
                *Maestro chiuda i bracci per favore
                */
                reporter.armingCheckFailure(
                    NavModes::All,                     // Blocca in tutti i modi
                    health_component_t::system,        // Componente logico
                    events::ID("check_arms_not_closed"),   // ID univoco per QGC
                    events::Log::Error, "Bracci non chiusi");              

                if (reporter.mavlink_log_pub()) {
                    mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Bracci non chiusi");
                }
            }

        } else {
            // Nessun messaggio ricevuto → errore di comunicazione
            /* EVENT
            */
            reporter.armingCheckFailure(
                NavModes::All,
                health_component_t::system,
                events::ID("check_arms_missing"),
                events::Log::Error,
                "Nessun dato da ESP32 stick");

            if (reporter.mavlink_log_pub()) {
                mavlink_log_critical(reporter.mavlink_log_pub(),
                    "Preflight Fail: Nessun dato da ESP32 stick");
            }
        }
    }
}
