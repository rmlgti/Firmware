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


OSD::OSD(int bus, int dev) :
	SPI("OSD", OSD_DEVICE_PATH, bus, dev, SPIDEV_MODE0, OSD_SPI_BUS_SPEED),
	_sensor_ok(false),
	_measure_ticks(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "osd_read")),
	_comms_errors(perf_alloc(PC_COUNT, "osd_com_err")),
	_battery_sub(-1),
	_on(false),
	_battery_voltage_filtered_v(0),
	_battery_discharge_mah(0)
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
		return PX4_ERROR;
	}

	_battery_sub = orb_subscribe(ORB_ID(battery_status));

	_sensor_ok = true;

	return PX4_OK;

}


int
OSD::probe()
{
	uint8_t data[2] = { 0, 0 };

	int ret = OK;

	// readRegister(0x00, &data[0], 1); // chip id

	// // Test the SPI communication, checking chipId and inverse chipId
	// if (data[0] == 0x49) {
	// 	return OK;
	// }

	ret &= writeRegister(0x00, 0x00); //disable video output

	usleep(1000);

	ret = readRegister(0xEC, &data[0], 1);
	usleep(1000);
	uint8_t x = data[0] & 0xEF;
	// ret &= writeRegister(0x6C, x);
	// usleep(1000);
	printf("probe read: %u\n", data[0]);

	// for(uint8_t i = 0; i<10; i++){
	// 	ret = readRegister(i, &data[0], 1);
	// 	printf("probe read[%u]: %u\n", (unsigned)i, data[0]);
	// }


	usleep(1000);
	x *= 1;

	ret &= writeRegister(0x04, 0); //DMM set to 0

	x = OSD_CHARS_PER_ROW + 1;

	ret &= writeRegister(0x05, 0x00); //DMAH

	ret &= writeRegister(0x06, x); //DMAL
	ret &= writeRegister(0x07, 146);

	// ret &= writeRegister(0x06, x + 1); //DMAL
	// ret &= writeRegister(0x07, 'A');

	// ret &= writeRegister(0x06, x + 2); //DMAL
	// ret &= writeRegister(0x07, 'T');

	// x = (OSD_CHARS_PER_ROW * 2) + 1;

	// ret &= writeRegister(0x06, x); //DMAL
	// ret &= writeRegister(0x07, 'C');

	// ret &= writeRegister(0x06, x + 1); //DMAL
	// ret &= writeRegister(0x07, 'U');

	// ret &= writeRegister(0x06, x + 2); //DMAL
	// ret &= writeRegister(0x07, 'R');

	// for(uint8_t i = 0; i< 255; i++){
	// 	int z = 30 + i;
	// 	if(z > 255){
	// 		ret &= writeRegister(0x05, 1); //DMAL
	// 		ret &= writeRegister(0x06, 30 + i - 255); //DMAL
	// 	} else {
	// 		ret &= writeRegister(0x06, 30 + i); //DMAL
	// 	}

	// 	ret &= writeRegister(0x07, i);
	// }

	// ret &= writeRegister(0x06, 30); //DMAL
	// ret &= writeRegister(0x07, 255);

	ret &= writeRegister(0x00, 0x08); //enable video output

	if (ret != OK) {
		printf("probe error\n");
		return PX4_ERROR;
	}

	_on = true;

	// not found on any address
	// return -EIO;
	return PX4_OK;
}


int
OSD::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			case 0:
				return -EINVAL;

			case SENSOR_POLLRATE_DEFAULT: {

					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(OSD_SAMPLE_RATE);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			case SENSOR_POLLRATE_MANUAL: {

					stop();
					_measure_ticks = 0;
					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(OSD_SAMPLE_RATE)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

ssize_t
OSD::read(device::file_t *filp, char *buffer, size_t buflen)
{
	// unsigned count = buflen / sizeof(struct optical_flow_s);
	// struct optical_flow_s *rbuf = reinterpret_cast<struct optical_flow_s *>(buffer);
	int ret = 0;

	// /* buffer must be large enough */
	// if (count < 1) {
	// 	return -ENOSPC;
	// }

	// /* if automatic measurement is enabled */
	// if (_measure_ticks > 0) {

	// 	/*
	// 	 * While there is space in the caller's buffer, and reports, copy them.
	// 	 * Note that we may be pre-empted by the workq thread while we are doing this;
	// 	 * we are careful to avoid racing with them.
	// 	 */
	// 	while (count--) {
	// 		if (_reports->get(rbuf)) {
	// 			ret += sizeof(*rbuf);
	// 			rbuf++;
	// 		}
	// 	}

	// 	/* if there was no data, warn the caller */
	// 	return ret ? ret : -EAGAIN;
	// }

	// /* manual measurement - run one conversion */
	// do {
	// 	_reports->flush();

	// 	/* trigger a measurement */
	// 	if (OK != collect()) {
	// 		ret = -EIO;
	// 		break;
	// 	}

	// 	/* state machine will have generated a report, copy it out */
	// 	if (_reports->get(rbuf)) {
	// 		ret = sizeof(*rbuf);
	// 	}

	// } while (0);

	return ret;
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
OSD::update_topics()
{
	struct battery_status_s battery = {};
	bool updated = false;

	/* update battery subscriptions */
	orb_check(_battery_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_sub, &battery);

		// if (battery.connected) {
		_battery_voltage_filtered_v = battery.voltage_filtered_v;
		_battery_discharge_mah = battery.discharged_mah;
		// }
	}


	return PX4_OK;
}


int
OSD::update_screen()
{

	// if (_on) {
	// 	ret &= writeRegister(0x00, 0x00); //disable video output
	// 	_on = false;

	// } else {
	// 	ret &= writeRegister(0x00, 0x08); //enable video output
	// 	_on = true;
	// }

	int ret = 0;

	char buf[5];
	sprintf(buf, "%4.2f", (double)_battery_voltage_filtered_v);

	ret &= writeRegister(0x04, 0); //DMM set to 0

	uint8_t x = OSD_CHARS_PER_ROW + 2;

	ret &= writeRegister(0x05, 0x00); //DMAH

	for (int i = 0; i < 5; i++) {
		ret &= writeRegister(0x06, x + i); //DMAL
		ret &= writeRegister(0x07, buf[i]);
	}

	ret &= writeRegister(0x06, x + 4); //DMAL
	ret &= writeRegister(0x07, 'V');

	sprintf(buf, "%d", (int)_battery_discharge_mah);

	x = (OSD_CHARS_PER_ROW * 2) + 1;

	for (int i = 0; i < 5; i++) {
		ret &= writeRegister(0x06, x + i); //DMAL
		ret &= writeRegister(0x07, buf[i]);
	}

	ret &= writeRegister(0x06, x + 4); //DMAL
	ret &= writeRegister(0x07, 7);

	ret &= writeRegister(0x00, 0x08); //enable video output

	ret *= 1;


	return PX4_OK;

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

void
OSD::cycle_trampoline(void *arg)
{
	OSD *dev = (OSD *)arg;

	dev->cycle();
}

void
OSD::cycle()
{
	update_screen();

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(LPWORK,
		   &_work,
		   (worker_t)&OSD::cycle_trampoline,
		   this,
		   USEC2TICK(2000000));

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

void	start(int spi_bus, int spi_dev);
void	stop();
void	test();
void	reset();
void	info();
void	usage();


/**
 * Start the driver.
 */
void
start(int spi_bus, int spi_dev)
{
	int fd;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new OSD(spi_bus, spi_dev);

	if (g_dev == nullptr) {
		printf("1\n");
		goto fail;
	}

	if (OK != g_dev->init()) {
		printf("2\n");
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(OSD_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		printf("3\n");
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		printf("4\n");
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}


/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{

	struct optical_flow_s report;
	ssize_t sz;

	int fd = open(OSD_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'osd start' if the driver is not running)", OSD_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("ret: %d, expected: %d", sz, sizeof(report));
		err(1, "immediate acc read failed");
	}

	print_message(report);

	errx(0, "PASS");
}


/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * Print a little info about how to start/stop/use the driver
 */
void usage()
{
	PX4_INFO("usage: osd {start|test|reset|info'}");
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
	int spi_dev = OSD_SPIDEV;

	while ((ch = px4_getopt(argc, argv, "bd:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			spi_bus = (uint8_t)atoi(myoptarg);
			break;

		case 'd':
			spi_dev = (uint8_t)atoi(myoptarg);
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
		osd::start(spi_bus, spi_dev);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		osd::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		osd::test();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		osd::info();
	}

	osd::usage();
	return PX4_ERROR;
}
