#ifdef __PX4_NUTTX

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <poll.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>
#include <parameters/param.h>

#include <sys/ioctl.h>
#include <lib/mathlib/mathlib.h>

#include "drivers/drv_pwm_trigger.h"
#include "pwm.h"
#include "drivers/drv_pwm_output.h"
#include "drivers/drv_gpio.h"

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_tasks.h>
#include <px4_workqueue.h>

// TODO : make these parameters later
#define PWM_CAMERA_SHOOT 100
#define PWM_CAMERA_NEUTRAL 100

CameraInterfacePWM::CameraInterfacePWM():
	CameraInterface()
{
	get_pins();
	setup();
}

CameraInterfacePWM::~CameraInterfacePWM()
{
	// Deinitialise trigger channels
	up_pwm_trigger_deinit();
}

void CameraInterfacePWM::setup()
{
	// Precompute the bitmask for enabled channels
	uint8_t pin_bitmask = 0;

	// up_pwm_servo_set_rate_group_update(0, 30);
	// up_pwm_servo_set_rate_group_update(1, 30);
	// up_pwm_servo_set_rate_group_update(2, 30);

	// int fd = -1;
	// fd = ::open(PX4FMU_DEVICE_PATH, O_RDWR);

	// if (fd < 0) {
	// 	PX4_ERR("open fail");
	// 	return;
	// }

	// if (::ioctl(fd, PWM_SERVO_SET_UPDATE_RATE, 0) != 0) {
	// 	PX4_ERR("open fail");
	// 	return;
	// }

	// if (::ioctl(fd, PWM_SERVO_SET_SELECT_UPDATE_RATE, 1 << 4) != 0) {
	// 	PX4_ERR("open fail");
	// 	return;
	// }

	for (unsigned i = 0; i < arraySize(_pins); i++) {
		if (_pins[i] >= 0) {
			pin_bitmask |= (1 << _pins[i]);
		}
	}

	// Initialize and arm channels
	up_pwm_trigger_init(pin_bitmask);

	// Set neutral pulsewidths
	for (unsigned i = 0; i < arraySize(_pins); i++) {
		if (_pins[i] >= 0) {
			up_pwm_trigger_set(_pins[i], math::constrain(PWM_CAMERA_NEUTRAL, PWM_CAMERA_NEUTRAL, 2000));
		}
	}

}

void CameraInterfacePWM::trigger(bool trigger_on_true)
{
	for (unsigned i = 0; i < arraySize(_pins); i++) {
		if (_pins[i] >= 0) {
			// Set all valid pins to shoot or neutral levels
			up_pwm_trigger_set(_pins[i], math::constrain(trigger_on_true ? PWM_CAMERA_SHOOT : PWM_CAMERA_NEUTRAL, 0, 2000));
		}
	}
}

void CameraInterfacePWM::info()
{
	PX4_INFO("PWM trigger mode (generic), pins enabled : [%d][%d][%d][%d][%d][%d]",
		 _pins[5], _pins[4], _pins[3], _pins[2], _pins[1], _pins[0]);
}

#endif /* ifdef __PX4_NUTTX */
