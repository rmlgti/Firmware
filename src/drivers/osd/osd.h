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

#pragma once

/**
 * @file osd.h
 * @author Daniele Pettenuzzo
 *
 * Driver for the ATXXXX chip on the omnibus fcu connected via SPI.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>

#include <drivers/device/spi.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <float.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <parameters/param.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>

#include <board_config.h>

// unused so far
#include <drivers/device/ringbuffer.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/sensor_gyro.h>

/* Configuration Constants */
#ifdef PX4_SPI_BUS_OSD
#define OSD_BUS PX4_SPI_BUS_OSD
#else
#define OSD_BUS 0
#endif

#ifdef PX4_SPIDEV_OSD
#define OSD_SPIDEV PX4_SPIDEV_OSD
#else
#define OSD_SPIDEV 0
#endif

#define OSD_SPI_BUS_SPEED (2000000L) // 2MHz

// #define DIR_WRITE(a) ((a) | (1 << 7))
// #define DIR_READ(a) ((a) & 0x7f)

#define DIR_READ(a) ((a) | (1 << 7))
#define DIR_WRITE(a) ((a) & 0x7f)

#define OSD_DEVICE_PATH "/dev/osd"

#define OSD_US 1000 /*   1 ms */
#define OSD_SAMPLE_RATE 10000 /*  10 ms */

/* OSD Registers addresses */


#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class OSD : public device::SPI
{
public:
	OSD(int bus = OSD_BUS, int dev = OSD_SPIDEV);

	virtual ~OSD();

	virtual int init();

	virtual ssize_t read(device::file_t *filp, char *buffer, size_t buflen);

	virtual int ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_info();

protected:
	virtual int probe();

private:
	work_s _work;
	ringbuffer::RingBuffer *_reports;
	bool _sensor_ok;
	int _measure_ticks;
	int _class_instance;
	int _orb_class_instance;

	orb_advert_t _optical_flow_pub;
	orb_advert_t _subsystem_pub;

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;

	uint64_t _previous_collect_timestamp;

	int _flow_sum_x = 0;
	int _flow_sum_y = 0;
	uint64_t _flow_dt_sum_usec = 0;
	bool _on;


	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void stop();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void cycle();
	int	collect();

	int readRegister(unsigned reg, uint8_t *data, unsigned count);
	int writeRegister(unsigned reg, uint8_t data);

	// int sensorInit();
	// int readMotionCount(int16_t &deltaX, int16_t &deltaY);

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void cycle_trampoline(void *arg);


};
/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int osd_main(int argc, char *argv[]);