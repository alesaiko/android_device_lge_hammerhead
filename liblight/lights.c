/*
 * Copyright (C) 2013-2018, The Android Open Source Project.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *	http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG	"lights"
#include <cutils/log.h>

#include <string.h>
#include <malloc.h>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>

#include <hardware/lights.h>

/************************* Linux Kernel part START *************************/
#define unlikely(x)	__builtin_expect(!!(x), 0)
#define likely(x)	__builtin_expect(!!(x), 1)

#define IS_ERR_VALUE(x)	unlikely((x) >= (unsigned long)-4095)

static inline long IS_ERR_OR_NULL(const void *ptr)
{
	return unlikely(!ptr) || IS_ERR_VALUE((unsigned long)ptr);
}
/************************* Linux Kernel part END ***************************/

/* Paths to kernel LED driver's nodes */
#define LCD_FILE		"/sys/class/leds/lcd-backlight/brightness"
#define RED_LED_FILE		"/sys/class/leds/red/brightness"
#define GREEN_LED_FILE		"/sys/class/leds/green/brightness"
#define BLUE_LED_FILE		"/sys/class/leds/blue/brightness"
#define RED_TIMEOUT_FILE	"/sys/class/leds/red/on_off_ms"
#define GREEN_TIMEOUT_FILE	"/sys/class/leds/green/on_off_ms"
#define BLUE_TIMEOUT_FILE	"/sys/class/leds/blue/on_off_ms"
#define RGB_LOCKED_FILE		"/sys/class/leds/red/rgb_start"

/* Current set_light() call */
static int (*set_light) (struct light_device_t *dev,
			 const struct light_state_t *state);

/* Mutex that protects set_light() function above from concurrent calls */
static pthread_once_t pthread_init = PTHREAD_ONCE_INIT;
static pthread_mutex_t pth_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Higher byte is ignored by LightsHAL */
#define COLOR_MASK		(0x00FFFFFF)

/**
 * get_rgb() - get RGB part within color by shifting it.
 * 0x  00    FF    FF    FF
 *     ^     ^     ^     ^
 *     |     |     |     |
 *  unused  red  green  blue
 */
#define get_rgb(clr, sft)	((clr >> sft) & 0xFF)

/**
 * [0] --> Battery charging LED,
 * [1] --> Notification LED,
 * [2] --> Attention LED;
 */
enum led_id {
	BATTERY_LED = 0,
	NOTIFICATIONS_LED,
	ATTENTION_LED,
	NUM_LEDS /* This constant is used as a number of all supported modes */
};

/* Declare NUM_LEDS global configurations for IDs above */
static struct led_config {
	unsigned int color;
	int on_ms, off_ms;
} led_data[NUM_LEDS];

/**
 * init_globals() - initialize global LED entities.
 *
 * Currently this initialized POSIX threads mutex only.
 */
static void init_globals(void)
{
	/* Init POSIX threads mutex */
	pthread_mutex_init(&pth_mutex, NULL);
}

/**
 * write_int() - write an integer value to a file.
 * @path: path to a file to write a value to.
 * @val: integer value to be written.
 */
static inline int write_int(const char *path, int val)
{
	/*
	 * Integer value cannot exceed 11 characters in length (2^31 - 1 and a
	 * 1-byte sign). 2 bytes are reserved for special symbols '\n' and '\0'
	 *
	 * Error buffer is a standart string of 80-char cap.
	 */
	char buf[13] = { 0 }, err[80] = { 0 };
	int ret, fd, bytes;

	fd = open(path, O_WRONLY);
	if (IS_ERR_VALUE(fd)) {
		strerror_r(errno, err, sizeof(err));
		ALOGE("%s: Unable to open %s (%s)", __func__, path, err);
		return -errno;
	}

	/* Convert integer into a string and aquire a number of written bytes */
	bytes = snprintf(buf, sizeof(buf), "%d\n", val);

	ret = write(fd, buf, bytes);
	if (IS_ERR_VALUE(ret)) {
		strerror_r(errno, err, sizeof(err));
		ALOGE("%s: Unable to write to %s (%s)", __func__, path, err);
		/* Fall through */
	}

	close(fd);

	return IS_ERR_VALUE(ret) ? -errno : 0;
}

/**
 * write_on_off() - write on/off integers to a file.
 * @path: path to a file to write values to.
 * @on: integer on_ms value to be written.
 * @off: integer off_ms value to be written.
 */
static inline int write_on_off(const char *path, int on, int off)
{
	/*
	 * Integer value cannot exceed 11 characters in length (2^31 - 1 and a
	 * 1-byte sign). 3 bytes are reserved for special symbols ' ', '\n'
	 * and '\0'
	 *
	 * Error buffer is a standart string of 80-char cap.
	 */
	char buf[25] = { 0 }, err[80] = { 0 };
	int ret, fd, bytes;

	fd = open(path, O_WRONLY);
	if (IS_ERR_VALUE(fd)) {
		strerror_r(errno, err, sizeof(err));
		ALOGE("%s: Unable to open %s (%s)", __func__, path, err);
		return -errno;
	}

	/* Convert integer into a string and aquire a number of written bytes */
	bytes = snprintf(buf, sizeof(buf), "%d %d\n", on, off);

	ret = write(fd, buf, bytes);
	if (IS_ERR_VALUE(ret)) {
		strerror_r(errno, err, sizeof(err));
		ALOGE("%s: Unable to write to %s (%s)", __func__, path, err);
		/* Fall through */
	}

	close(fd);

	return IS_ERR_VALUE(ret) ? -errno : 0;
}

/**
 * write_leds_locked() - write all the data within LED config to a LED driver.
 * @led: pointer to led_config structure.
 *
 * ! This function must be called with POSIX threads mutex locked.
 */
static inline int write_leds_locked(const struct led_config *led)
{
	/**
	 * _get_rgb() - get a specific color within led_config::color variable.
	 * @shift: number of bits to shift.
	 */
#define _get_rgb(shift)		get_rgb(led->color, shift)

	int red = _get_rgb(16), green = _get_rgb(8), blue = _get_rgb(0);
	int ret = 0;

	/* Stop LED RGB first */
	ret |= write_int(RGB_LOCKED_FILE, 0);

	/* Write RGB colors to appropriate nodes */
	ret |= write_int(RED_LED_FILE, red);
	ret |= write_int(GREEN_LED_FILE, green);
	ret |= write_int(BLUE_LED_FILE, blue);

	/* Write on/off loop times */
	ret |= write_on_off(RED_TIMEOUT_FILE, led->on_ms, led->off_ms);
	ret |= write_on_off(GREEN_TIMEOUT_FILE, led->on_ms, led->off_ms);
	ret |= write_on_off(BLUE_TIMEOUT_FILE, led->on_ms, led->off_ms);

	/* Start adjusted LED RGB */
	ret |= write_int(RGB_LOCKED_FILE, 1);

	return ret;
}

/**
 * set_light_locked() - adjust current LED device.
 * @state: pointer to a constant light_state_t structure.
 * @type: identificator of a target LED mode.
 *
 * ! This function must be called with POSIX threads mutex locked.
 */
static inline int set_light_locked(const struct light_state_t *state, int type)
{
	struct led_config *led = &led_data[type], led_off;
	static int cur_type = -EINVAL;
	int red, green, blue, i;

	/* Used to completely disable LED */
	memset(&led_off, 0, sizeof(struct led_config));

	/* Load target times according to a requested flash mode */
	switch (state->flashMode) {
	case LIGHT_FLASH_TIMED:
	case LIGHT_FLASH_HARDWARE:
		led->on_ms = state->flashOnMS;
		led->off_ms = state->flashOffMS;
		break;
	default:
		led->on_ms = led->off_ms = 0;
		break;
	}

	led->color = state->color & COLOR_MASK;
	if (led->color && type >= cur_type) {
		/* Switch to a higher priority LED */
		cur_type = type;
		return write_leds_locked(led);
	} else if (type == cur_type) {
		/* Handle both update and disable cases */
		for (i = (type - 1); i >= 0; i--) {
			if (led_data[i].color) {
				cur_type = i;
				return write_leds_locked(&led_data[i]);
			}
		}
	}

	/* Disable LED if nothing requires it */
	cur_type = -EINVAL;

	return write_leds_locked(&led_off);
}

/**
 * rgb_to_brightness() - convert colors to brightness.
 * @state: pointer to a constant light_state_t structure.
 */
static inline int rgb_to_brightness(const struct light_state_t *state)
{
	int color = state->color & COLOR_MASK;

	return ((77  * get_rgb(color, 16)) +
		(150 * get_rgb(color, 8)) +
		(29  * get_rgb(color, 0))) >> 8;
}

static int set_light_backlight(struct light_device_t *dev __unused,
			       const struct light_state_t *state)
{
	int ret;

	pthread_mutex_lock(&pth_mutex);
	ret = write_int(LCD_FILE, rgb_to_brightness(state));
	pthread_mutex_unlock(&pth_mutex);

	return ret;
}

static int set_light_battery(struct light_device_t *dev __unused,
			     const struct light_state_t *state)
{
	int ret;

	pthread_mutex_lock(&pth_mutex);
	ret = set_light_locked(state, BATTERY_LED);
	pthread_mutex_unlock(&pth_mutex);

	return ret;
}

static int set_light_notifications(struct light_device_t *dev __unused,
				   const struct light_state_t *state)
{
	int ret;

	pthread_mutex_lock(&pth_mutex);
	ret = set_light_locked(state, NOTIFICATIONS_LED);
	pthread_mutex_unlock(&pth_mutex);

	return ret;
}

static int set_light_attention(struct light_device_t *dev __unused,
			       const struct light_state_t *state)
{
	struct light_state_t fixed;
	int ret;

	pthread_mutex_lock(&pth_mutex);
	memcpy(&fixed, state, sizeof(struct light_state_t));

	switch (fixed.flashMode) {
	case LIGHT_FLASH_NONE:
		fixed.color = 0;
		break;
	case LIGHT_FLASH_HARDWARE:
		if (fixed.flashOnMS > 0 && !fixed.flashOffMS)
			fixed.flashMode = LIGHT_FLASH_NONE;
		break;
	}

	ret = set_light_locked(&fixed, ATTENTION_LED);
	pthread_mutex_unlock(&pth_mutex);

	return ret;
}

static int close_lights(struct light_device_t *dev)
{
	if (likely(dev))
		free(dev);

	return 0;
}

static int open_lights(const struct hw_module_t *module,
		       const char *name, struct hw_device_t **device)
{
	struct light_device_t *dev;

	/**
	 * is_requested() - check whether an ID was requested by a caller.
	 * @id: requsted ID.
	 */
#define is_requested(id) !strncmp(name, id, 32)

	/* Select set_light() call according to a requested identificator */
	if (is_requested(LIGHT_ID_BACKLIGHT))
		set_light = set_light_backlight;
	else if (is_requested(LIGHT_ID_BATTERY))
		set_light = set_light_battery;
	else if (is_requested(LIGHT_ID_NOTIFICATIONS))
		set_light = set_light_notifications;
	else if (is_requested(LIGHT_ID_ATTENTION))
		set_light = set_light_attention;
	else
		return -EINVAL;

	/* Initialize pthread lock */
	pthread_once(&pthread_init, init_globals);

	/* Allocate enough memory for NUM_LEDS entities */
	dev = calloc(NUM_LEDS, sizeof(*dev));
	if (IS_ERR_OR_NULL(dev))
		return -ENOMEM;

	dev->common.tag = HARDWARE_DEVICE_TAG;
	dev->common.version = LIGHTS_DEVICE_API_VERSION_1_0;
	dev->common.module = (struct hw_module_t *)module;
	dev->common.close = (int (*)(struct hw_device_t *dev))close_lights;
	dev->set_light = set_light;

	*device = (struct hw_device_t *)dev;

	return 0;
}

static struct hw_module_methods_t lights_module_methods = {
	.open = open_lights,
};

struct hw_module_t HAL_MODULE_INFO_SYM = {
	.tag			= HARDWARE_MODULE_TAG,
	.module_api_version	= HARDWARE_MODULE_API_VERSION(1, 0),
	.hal_api_version	= HARDWARE_HAL_API_VERSION,
	.id			= LIGHTS_HARDWARE_MODULE_ID,
	.name			= "lights Module",
	.author			= "Google, Inc.",
	.methods		= &lights_module_methods,
};
