/****************************************************************************
 * Simple I2C driver for custom ESP32-based digital stick (two buttons)
 * Patterned after PX4's rgbled.cpp I2C driver template.
 ****************************************************************************/

#include <string.h>

#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/arms_status.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

/* ===== Configuration ===== */
#define MODULE_NAME              "esp32_stick_i2c"
#define ESP32_STICK_I2C_ADDR    0x42     // Slave address of your ESP32
#define ESP32_CMD_BOTH          0x03     // Command: return both buttons in 1 byte

class ESP32Stick : public device::I2C, public I2CSPIDriver<ESP32Stick>
{
public:
	ESP32Stick(const I2CSPIDriverConfig &config);
	~ESP32Stick() override = default;

	static void print_usage();

	int  init() override;
	int  probe() override;
	void RunImpl();

private:
	void print_status() override;

	/* Low-level helpers */
	int  write_cmd(uint8_t cmd);
	int  read_byte(uint8_t &val);
	int  read_state(uint8_t &state);   // WRITE (cmd) then READ (1 byte)

	/* Debug/perf */
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": read")};
	perf_counter_t _comms_errs{perf_alloc(PC_COUNT,   MODULE_NAME ": comms err")};

	/* Last state cached (for status) */
	uint8_t _last_state{0xFF};

	uORB::Publication<arms_status_s> _arms_pub{ORB_ID(arms_status)};

};

ESP32Stick::ESP32Stick(const I2CSPIDriverConfig &config)
: I2C(config), I2CSPIDriver(config)
{
}

int ESP32Stick::init() {
    int ret = I2C::init();
    if (ret != OK) return ret;
    PX4_INFO("init OK (addr 0x%02X)", get_device_address());
    ScheduleOnInterval(200_ms);
    return OK;
}


/* Probe: fai la stessa sequenza del runtime (WRITE cmd, poi READ 1 byte) */
int ESP32Stick::probe()
{
	uint8_t state = 0;
	int ret = read_state(state);
	return (ret == OK) ? OK : PX4_ERROR;
	return OK;
}

void ESP32Stick::RunImpl()
{
	perf_begin(_sample_perf);

	uint8_t state = 0;
	const int ret = read_state(state);

	if (ret == OK) {
		_last_state = state;

		 // Prepara messaggio uORB
		arms_status_s msg{};
		msg.timestamp = hrt_absolute_time();
		msg.button1 = (state & 0x01) ? 1 : 0;
		msg.button2 = (state & 0x02) ? 1 : 0;

		_arms_pub.publish(msg);


	} else {
		perf_count(_comms_errs);
		PX4_WARN("I2C read failed");
	}

	perf_end(_sample_perf);

	/* Ripianifica alla prossima cadenza */
	ScheduleOnInterval(1000_ms);
}

void ESP32Stick::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errs);
	PX4_INFO("Last state cached: 0x%02X (B1=%d B2=%d)",
	         _last_state, (_last_state & 0x01) != 0, (_last_state & 0x02) != 0);
}

/* ===== Low-level I2C helpers ===== */

int ESP32Stick::write_cmd(uint8_t cmd)
{
	/* write only */
	return transfer(&cmd, 1, nullptr, 0);
}

int ESP32Stick::read_byte(uint8_t &val)
{
	/* read only */
	return transfer(nullptr, 0, &val, 1);
}

int ESP32Stick::read_state(uint8_t &state)
{
	/* Sequenza robusta per ESP32 slave: WRITE (cmd) -> READ (1 byte) */
	int ret = write_cmd(ESP32_CMD_BOTH);
	if (ret != OK) {
		return ret;
	}
	ret = read_byte(state);
	return ret;
}

/* ===== CLI / Module glue ===== */

void ESP32Stick::print_usage()
{
	PRINT_MODULE_USAGE_NAME(MODULE_NAME, "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);   // I2C only
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(ESP32_STICK_I2C_ADDR);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int esp32_stick_i2c_main(int argc, char *argv[])
{
	using ThisDriver = ESP32Stick;
	BusCLIArguments cli{true, false};           // I2C only
	cli.default_i2c_frequency = 100000;         // inizia a 100 kHz (più tollerante), poi puoi salire
	cli.i2c_address = ESP32_STICK_I2C_ADDR;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	/* Devtype opzionale; 0 va bene per mod custom */
	BusInstanceIterator iterator(MODULE_NAME, cli, 0);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
