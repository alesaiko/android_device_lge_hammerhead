/*
 * Copyright (C) 2014-2018, The Android Open Source Project.
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

#define LOG_TAG	"PowerHAL"
#include <utils/Log.h>

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>

#include <sys/un.h>
#include <sys/poll.h>
#include <sys/socket.h>

#include <linux/netlink.h>
#include <hardware/power.h>

/************************* Linux Kernel part START *************************/
#define __read_mostly	__attribute__((__section__(".data..read_mostly")))

#define unlikely(x)	__builtin_expect(!!(x), 0)
#define likely(x)	__builtin_expect(!!(x), 1)

#define IS_ERR_VALUE(x)	unlikely((x) >= (unsigned long)-4095)

static inline long IS_ERR_OR_NULL(const void *ptr)
{
	return unlikely(!ptr) || IS_ERR_VALUE((unsigned long)ptr);
}
/************************* Linux Kernel part END ***************************/

/* Requested states for encoding hint */
#define STATE_OFF			"state=0"
#define STATE_ON			"state=1"
#define STATE_HDR_ON			"state=2"
#define STATE_HDR_OFF			"state=3"

/* Boost socket to be connected to client address */
#define BOOST_SOCKET			"/dev/socket/pb"

/* Maximum length of a data chunk to be send to socket */
#define MAX_LENGTH			(50)

/* Uevent message properties */
#define UEVENT_MSG_LEN			(2048)
#define UEVENT_STRING			"online@/devices/system/cpu/"

/* Maximum number of retries to change frequencies during LPM enable/disable */
#define NUM_RETRIES			(20)

/* Time in usecs to wait between every retry above */
#define SLEEP_BETWEEN_RETRIES_USEC	(200)

/* Frequency bounds during LPM mode */
#define LOW_POWER_MIN_FREQ		"300000"
#define LOW_POWER_MAX_FREQ		"1036800"

/* Frequency bounds during normal mode */
#define NORMAL_MIN_FREQ			"300000"
#define NORMAL_MAX_FREQ			"2457600"

/* Macros to improve readability */
#define for_each_possible_cpu(cpu) \
	for ((cpu) = 0; (cpu) < NUM_CPUS; (cpu)++)

/* Number of cpus (Hammerhead --> QCOM MSM8974 4xKrait 400) */
enum cpus {
	CPU0 = 0,
	CPU1,
	CPU2,
	CPU3,
	NUM_CPUS /* This constant is used as a number of all cpus */
};

/* Socket file descriptor */
static int __read_mostly client_sockfd;

/* Socket address to send messages to */
static struct sockaddr_un client_addr;

/* Polling descriptor for uevent handler */
static struct pollfd pfd;

/* Paths to lower CPU frequency bound */
static const char *cpufreq_min[NUM_CPUS] __read_mostly = {
	[CPU0] = "/sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq",
	[CPU1] = "/sys/devices/system/cpu/cpu1/cpufreq/scaling_min_freq",
	[CPU2] = "/sys/devices/system/cpu/cpu2/cpufreq/scaling_min_freq",
	[CPU3] = "/sys/devices/system/cpu/cpu3/cpufreq/scaling_min_freq",
};

/* Paths to upper CPU frequency bound */
static const char *cpufreq_max[NUM_CPUS] __read_mostly = {
	[CPU0] = "/sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq",
	[CPU1] = "/sys/devices/system/cpu/cpu1/cpufreq/scaling_max_freq",
	[CPU2] = "/sys/devices/system/cpu/cpu2/cpufreq/scaling_max_freq",
	[CPU3] = "/sys/devices/system/cpu/cpu3/cpufreq/scaling_max_freq",
};

/* Boolean to indicate that global LPM is currently active */
static bool low_power_mode;

/* Boolean to indicate that LPM is currently active on a CPU */
static bool cpu_lpm_enabled[NUM_CPUS];

/* Mutex that protects LPM mode enable/disable routines */
static pthread_mutex_t lpm_lock = PTHREAD_MUTEX_INITIALIZER;

static inline void socket_init(void)
{
	/* Return early if socket descriptor is already initialized */
	if (client_sockfd)
		return;

	/* Create socket descriptor */
	client_sockfd = socket(PF_UNIX, SOCK_DGRAM, 0);
	if (IS_ERR_VALUE(client_sockfd)) {
		ALOGE("%s: Unable to create socket descriptor", __func__);
		return;
	}

	/* Zero memory first */
	memset(&client_addr, 0, sizeof(struct sockaddr_un));

	/* Set up client address */
	client_addr.sun_family = AF_UNIX;
	snprintf(client_addr.sun_path, UNIX_PATH_MAX, BOOST_SOCKET);
}

/**
 * write_string() - write a string to a file.
 * @path: path to a file to write a string to.
 * @str: a string to be written.
 */
static inline int write_string(const char *path, const char *str)
{
	char err[80] = { 0 };
	int ret, fd;

	fd = open(path, O_WRONLY);
	if (IS_ERR_VALUE(fd)) {
		strerror_r(errno, err, sizeof(err));
		ALOGE("%s: Unable to open %s (%s)", __func__, path, err);
		return -errno;
	}

	ret = write(fd, str, strlen(str));
	if (IS_ERR_VALUE(ret)) {
		strerror_r(errno, err, sizeof(err));
		ALOGE("%s: Unable to write to %s (%s)", __func__, path, err);
		/* Fall through */
	}

	close(fd);

	return IS_ERR_VALUE(ret) ? -errno : 0;
}

static inline void try_to_enable_lpm(int cpu, int nretries)
{
	int ret, retry = nretries;

	/* Return early if LPM on cpu is already enabled */
	if (unlikely(cpu_lpm_enabled[cpu]))
		return;

	/* Try at least once change freqs */
	do {
		/* We don't bother checking the correction of min freq change */
		write_string(cpufreq_min[cpu], LOW_POWER_MIN_FREQ);

		ret = write_string(cpufreq_max[cpu], LOW_POWER_MAX_FREQ);
		if (likely(!ret)) {
			cpu_lpm_enabled[cpu] = true;
			break;
		}

		usleep(SLEEP_BETWEEN_RETRIES_USEC);
	} while (--retry);
}

static inline void try_to_disable_lpm(int cpu, int nretries)
{
	int ret, retry = nretries;

	/* Return early if LPM on cpu is already disabled */
	if (unlikely(!cpu_lpm_enabled[cpu]))
		return;

	/* Try at least once change freqs */
	do {
		/* We don't bother checking the correction of min freq change */
		write_string(cpufreq_min[cpu], NORMAL_MIN_FREQ);

		ret = write_string(cpufreq_max[cpu], NORMAL_MAX_FREQ);
		if (likely(!ret)) {
			cpu_lpm_enabled[cpu] = false;
			break;
		}

		usleep(SLEEP_BETWEEN_RETRIES_USEC);
	} while (--retry);
}

static inline int uevent_handler(void)
{
	char msg[UEVENT_MSG_LEN] = { 0 }, *cp;
	int cpu, len;

	/* Try to recieve message from polling descriptor */
	len = recv(pfd.fd, msg, UEVENT_MSG_LEN, MSG_DONTWAIT);
	if (len < 1 || len >= UEVENT_MSG_LEN)
		return -EINVAL;

	cp = msg;
	/* Try to find UEVENT_STRING within a recieved message */
	if (IS_ERR_OR_NULL(strstr(cp, UEVENT_STRING)))
		return -EINVAL;

	/* Last character in a string is a CPU number */
	cpu = strtol(cp + strlen(cp) - 1, NULL, 10);
	if (cpu < 0 || cpu >= NUM_CPUS || errno == EINVAL || errno == ERANGE)
		return -EINVAL;

	pthread_mutex_lock(&lpm_lock);
	low_power_mode ? try_to_enable_lpm(cpu, NUM_RETRIES) :
			 try_to_disable_lpm(cpu, NUM_RETRIES);
	pthread_mutex_unlock(&lpm_lock);

	return 0;
}

static void *thread_uevent(void *x __unused)
{
	int ret;

	while (1) {
		ret = poll(&pfd, 1, -1);
		if (IS_ERR_VALUE(ret)) {
			if (likely(errno == EINTR))
				continue;

			ALOGE("%s: Unable to poll", __func__);
			break;
		}

		ret = uevent_handler();
		if (IS_ERR_VALUE(ret))
			ALOGE("%s: Unable to process uevent", __func__);
	}

	return NULL;
}

static inline void uevent_init(void)
{
	struct sockaddr_nl client;
	pthread_t tid;

	/* Create polling descriptor */
	pfd.fd = socket(PF_NETLINK, SOCK_DGRAM, NETLINK_KOBJECT_UEVENT);
	if (IS_ERR_VALUE(pfd.fd)) {
		ALOGE("%s: Unable to create polling descriptor", __func__);
		return;
	}

	/* Create POSIX threads and bind uevent to them */
	pthread_create(&tid, NULL, thread_uevent, NULL);

	/* Set up socket client */
	client = (struct sockaddr_nl) {
		.nl_family = AF_NETLINK,
		.nl_pid = tid,
		.nl_groups = -1,
	};

	/* Bind socket address to polling descriptor */
	pfd.events = POLLIN;
	bind(pfd.fd, (void *)&client, sizeof(struct sockaddr_nl));
}

static void power_init(struct power_module *module __unused)
{
	ALOGI("%s", __func__);

	socket_init();
	uevent_init();
}

static inline int send_to_socket(const char *data)
{
	return sendto(client_sockfd, data, strlen(data), 0,
		     (const struct sockaddr *)&client_addr,
		      sizeof(struct sockaddr_un));
}

static inline void touch_boost(void)
{
	char data[MAX_LENGTH] = { 0 };
	pid_t client = getpid();

	if (IS_ERR_VALUE(client_sockfd)) {
		ALOGE("%s: Unable to open socket descriptor", __func__);
		return;
	}

	snprintf(data, MAX_LENGTH, "1:%d", client);
	send_to_socket(data);
}

static inline void sync_thread(bool enable)
{
	char data[MAX_LENGTH] = { 0 };
	pid_t client = getpid();

	if (IS_ERR_VALUE(client_sockfd)) {
		ALOGE("%s: Unable to open socket descriptor", __func__);
		return;
	}

	snprintf(data, MAX_LENGTH, (enable ? "2:%d" : "3:%d"), client);
	send_to_socket(data);
}

static inline void enc_boost(bool enable)
{
	char data[MAX_LENGTH] = { 0 };
	pid_t client = getpid();

	if (IS_ERR_VALUE(client_sockfd)) {
		ALOGE("%s: Unable to open socket descriptor", __func__);
		return;
	}

	snprintf(data, MAX_LENGTH, (enable ? "5:%d" : "6:%d"), client);
	send_to_socket(data);
}

static inline void process_video_encode_hint(void *data)
{
	char *str = data;

	socket_init();
	if (IS_ERR_VALUE(client_sockfd)) {
		ALOGE("%s: Unable to open socket descriptor", __func__);
		return;
	}

	/* Return early if data is absent */
	if (IS_ERR_OR_NULL(str))
		return;

	/**
	 * is_requested() - check whether a state was requested by a caller.
	 * @state: requsted state.
	 */
#define is_requested(state) !strncmp(str, state, strlen(state))

	/* Select boosting according to a requsted boost state */
	if (is_requested(STATE_ON)) {
		/* Video encode started */
		sync_thread(false);
		enc_boost(false);
	} else if (is_requested(STATE_OFF)) {
		/* Video encode stopped */
		sync_thread(true);
		enc_boost(true);
	} else if (is_requested(STATE_HDR_ON)) {
		/* HDR usecase started */
		sync_thread(false);
	} else if (is_requested(STATE_HDR_OFF)) {
		/* HDR usecase stopped */
		sync_thread(true);
	}
}

static void
power_set_interactive(struct power_module *module __unused, int enable)
{
	static int last_state = 0;

	/* Return early if the same state was requested */
	if (last_state == enable)
		return;

	last_state = enable;
	ALOGV("%s: %s", __func__, (enable ? "ON" : "OFF"));

	if (enable) {
		sync_thread(true);
		touch_boost();
	} else {
		sync_thread(false);
	}
}

/**
 * enable_low_power_mode() - try to enable LPM for all cpus.
 */
static inline void enable_low_power_mode(void)
{
	int ret, cpu;

	/* Return early if LPM is already enabled */
	if (unlikely(low_power_mode))
		return;

	low_power_mode = true;
	for_each_possible_cpu(cpu)
		try_to_enable_lpm(cpu, 0);
}

/**
 * disable_low_power_mode() - try to disable LPM for all cpus.
 */
static inline void disable_low_power_mode(void)
{
	int ret, cpu;

	/* Return early if LPM is already disabled */
	if (unlikely(!low_power_mode))
		return;

	low_power_mode = false;
	for_each_possible_cpu(cpu)
		try_to_disable_lpm(cpu, 0);
}

static void power_hint(struct power_module *module __unused,
		       power_hint_t hint, void *data)
{
	int lpm = (int)data;

	switch (hint) {
	case POWER_HINT_INTERACTION:
		touch_boost();
		break;
	case POWER_HINT_VIDEO_ENCODE:
		process_video_encode_hint(data);
		break;
	case POWER_HINT_LOW_POWER:
		pthread_mutex_lock(&lpm_lock);
		lpm ? enable_low_power_mode() :
		      disable_low_power_mode();
		pthread_mutex_unlock(&lpm_lock);
		break;
	}
}

static struct hw_module_methods_t power_module_methods = {
	.open = NULL,
};

struct power_module HAL_MODULE_INFO_SYM = {
	.common = {
		.tag			= HARDWARE_MODULE_TAG,
		.module_api_version	= POWER_MODULE_API_VERSION_0_2,
		.hal_api_version	= HARDWARE_HAL_API_VERSION,
		.id			= POWER_HARDWARE_MODULE_ID,
		.name			= "Hammerhead Power HAL",
		.author			= "The Android Open Source Project",
		.methods		= &power_module_methods,
	},
	.init		= power_init,
	.setInteractive	= power_set_interactive,
	.powerHint	= power_hint,
};
