/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file osd.cpp
 * @author Daniele Pettenuzzo
 *
 * Driver for the ATXXXX chip on the omnibus fcu connected via SPI.
 */

#include "osd.h"


OSD::OSD(int bus) :
	SPI("OSD", OSD_DEVICE_PATH, bus, OSD_SPIDEV, SPIDEV_MODE0, OSD_SPI_BUS_SPEED),
	_measure_ticks(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "osd_read")),
	_comms_errors(perf_alloc(PC_COUNT, "osd_com_err")),
	_battery_sub(-1),
	_local_position_sub(-1),
	_battery_voltage_filtered_v(0),
	_battery_discharge_mah(0),
	_battery_valid(false),
	_local_position_z(0),
	_local_position_valid(false)
{

	// enable debug() calls
	_debug_enabled = false;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

OSD::~OSD()
{
	/* make sure we are truly inactive */
	stop();

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}


int
OSD::init()
{
	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		goto fail;
	}

	if (reset() != PX4_OK) {
		goto fail;
	}

	if (writeRegister(0x00, 0x48) != PX4_OK) { //DMM set to 0
		goto fail;
	}

	if (writeRegister(0x04, 0) != PX4_OK) { //DMM set to 0
		goto fail;
	}

	_battery_sub = orb_subscribe(ORB_ID(battery_status));
	_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	return PX4_OK;

fail:
	return PX4_ERROR;

}


int
OSD::probe()
{
	uint8_t data = 0;
	int ret = PX4_OK;

	ret |= writeRegister(0x00, 0x01); //disable video output

	// ret |= readRegister(0xEC, &data[0], 1);
	ret |= readRegister(0x00, &data, 1);

	if (data != 1 || ret != PX4_OK) {
		printf("probe error\n");
	}

	return ret;
}


int
OSD::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	return -1;
}

ssize_t
OSD::read(device::file_t *filp, char *buffer, size_t buflen)
{
	return -1;
}


int
OSD::readRegister(unsigned reg, uint8_t *data, unsigned count)
{
	uint8_t cmd[5]; 					// read up to 4 bytes
	int ret;

	cmd[0] = DIR_READ(reg);

	ret = transfer(&cmd[0], &cmd[0], count + 1);

	if (OK != ret) {
		perf_count(_comms_errors);
		DEVICE_LOG("spi::transfer returned %d", ret);
		return ret;
	}

	memcpy(&data[0], &cmd[1], count);

	return ret;

}


int
OSD::writeRegister(unsigned reg, uint8_t data)
{
	uint8_t cmd[2]; 						// write 1 byte
	int ret;

	cmd[0] = DIR_WRITE(reg);
	cmd[1] = data;

	ret = transfer(&cmd[0], nullptr, 2);

	if (OK != ret) {
		perf_count(_comms_errors);
		DEVICE_LOG("spi::transfer returned %d", ret);
		return ret;
	}

	return ret;

}

int
OSD::add_character_to_screen(char c, uint8_t pos_x, uint8_t pos_y)
{

	uint16_t position = (OSD_CHARS_PER_ROW * pos_y) + pos_x;
	uint8_t position_lsb;
	int ret = PX4_OK;

	if (position > 0xFF) {
		position_lsb = static_cast<uint8_t>(position) - 0xFF;
		ret |= writeRegister(0x05, 0x01); //DMAH

	} else {
		position_lsb = static_cast<uint8_t>(position);
		ret |= writeRegister(0x05, 0x00); //DMAH
	}

	ret |= writeRegister(0x06, position_lsb); //DMAL
	ret |= writeRegister(0x07, c);

	return ret;
}

int
OSD::add_battery_symbol(uint8_t pos_x, uint8_t pos_y)
{
	return add_character_to_screen(146, pos_x, pos_y);
}

int
OSD::add_battery_info(uint8_t pos_x, uint8_t pos_y)
{

	char buf[5];
	int ret = PX4_OK;

	sprintf(buf, "%4.2f", (double)_battery_voltage_filtered_v);

	for (int i = 0; i < 5; i++) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	ret |= add_character_to_screen('V', pos_x + 5, pos_y);

	pos_y++;

	sprintf(buf, "%4d", (int)_battery_discharge_mah);

	for (int i = 0; i < 5; i++) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	ret |= add_character_to_screen(7, pos_x + 5, pos_y); // mAh symbol

	return ret;
}

int
OSD::add_altitude_symbol(uint8_t pos_x, uint8_t pos_y)
{
	// return add_character_to_screen(xxx, pos_x, pos_y);
	return PX4_OK;
}

int
OSD::add_altitude(uint8_t pos_x, uint8_t pos_y)
{

	return PX4_OK;
}

int
OSD::enable_screen()
{
	uint8_t data;
	int ret = PX4_OK;

	ret |= readRegister(0x00, &data, 1);
	ret |= writeRegister(0x00, data | 0x48);

	return ret;
}

int
OSD::disable_screen()
{
	uint8_t data;
	int ret = PX4_OK;

	ret |= readRegister(0x00, &data, 1);
	ret |= writeRegister(0x00, data & 0xF7);

	return ret;
}


int
OSD::update_topics()//TODO have an argument to choose what to update and return validity
{
	struct battery_status_s battery = {};
	struct vehicle_local_position_s local_position = {};
	bool updated = false;

	/* update battery subscriptions */
	orb_check(_battery_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_sub, &battery);

		if (battery.connected) {
			_battery_voltage_filtered_v = battery.voltage_filtered_v;
			_battery_discharge_mah = battery.discharged_mah;
			_battery_valid = true;

		} else {
			_battery_valid = false;
		}
	}

	/* update vehicle local position subscriptions */
	orb_check(_local_position_sub, &updated);

	if (updated) {
		if (local_position.z_valid) {
			_local_position_z = -local_position.z;
			_local_position_valid = true;

		} else {
			_local_position_valid = false;
		}
	}

	return PX4_OK;
}


int
OSD::update_screen()
{
	int ret = PX4_OK;

	if (_battery_valid) {
		ret |= add_battery_symbol(1, 1);
		ret |= add_battery_info(2, 1);
	}

	// if(_local_position_valid){
	// 	ret |= add_altitude_symbol(1, 2);
	// 	ret |= add_altitude(2, 2);
	// }

	// enable_screen();

	return ret;

}


void
OSD::start()
{
	/* schedule a cycle to start things */
	work_queue(LPWORK, &_work, (worker_t)&OSD::cycle_trampoline, this, USEC2TICK(OSD_US));
}

void
OSD::stop()
{
	work_cancel(LPWORK, &_work);
}

int
OSD::reset()
{
	int ret = writeRegister(0x00, 0x02);
	usleep(100);

	return ret;
}

void
OSD::cycle_trampoline(void *arg)
{
	OSD *dev = (OSD *)arg;

	dev->cycle();
}

void
OSD::cycle()
{
	update_topics();

	if (_battery_valid || _local_position_valid) {
		update_screen();
	}

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(LPWORK,
		   &_work,
		   (worker_t)&OSD::cycle_trampoline,
		   this,
		   USEC2TICK(OSD_UPDATE_RATE));

}

void
OSD::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("battery_status: %.3f\n", (double)_battery_voltage_filtered_v);
}


/**
 * Local functions in support of the shell command.
 */
namespace osd
{

OSD	*g_dev;

int	start(int spi_bus);
int	stop();
int	info();
void usage();


/**
 * Start the driver.
 */
int
start(int spi_bus)
{
	int fd;

	if (g_dev != nullptr) {
		PX4_ERR("already started");
		goto fail;
	}

	/* create the driver */
	g_dev = new OSD(spi_bus);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	fd = open(OSD_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	g_dev->start();

	return PX4_OK;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_ERR("driver start failed");
	return PX4_ERROR;
}

/**
 * Stop the driver
 */
int
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return PX4_OK;
}

/**
 * Print a little info about how to start/stop/use the driver
 */
void usage()
{
	PX4_INFO("usage: osd {start|stop|status'}");
	PX4_INFO("    [-b SPI_BUS]");
}

} // namespace osd


int
osd_main(int argc, char *argv[])
{
	if (argc < 2) {
		osd::usage();
		return PX4_ERROR;
	}

	// don't exit from getopt loop to leave getopt global variables in consistent state,
	// set error flag instead
	bool err_flag = false;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int spi_bus = OSD_BUS;

	while ((ch = px4_getopt(argc, argv, "b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			spi_bus = (uint8_t)atoi(myoptarg);
			break;

		default:
			err_flag = true;
			break;
		}
	}

	if (err_flag) {
		osd::usage();
		return PX4_ERROR;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		return osd::start(spi_bus);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return osd::stop();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return osd::info();
	}

	osd::usage();
	return PX4_ERROR;
}
