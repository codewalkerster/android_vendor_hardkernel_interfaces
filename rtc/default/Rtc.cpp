/*
 * Copyright (C) 2019 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Rtc.h"

#include <android-base/logging.h>
#include <linux/rtc.h>
#include <log/log.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#ifndef __aarch64__
#include <time64.h>
#endif

using namespace std;

namespace aidl {
namespace vendor {
namespace hardkernel{
namespace hardware {
namespace rtc {

#define WAKEALARM_PATH "/sys/class/rtc/rtc0/wakealarm"
#define RTC_PATH "/dev/rtc0"

static int write_sys(const char* path, long value) {
    int fd;

    fd = open(path, O_RDWR);
    if (fd >= 0) {
        char buf[20];
        int bytes = snprintf(buf, sizeof(buf), "%ld\n", value);
        ssize_t amt = write(fd, buf, (size_t)bytes);
        close(fd);
        return amt == -1? -errno : 0;
    } else {
        ALOGE("write_int() failed to open %s:%s\n", path, strerror(errno));
        return -errno;
    }
}

::ndk::ScopedAStatus Rtc::getTime(std::string *_aidl_return) {
    int fd = open(RTC_PATH, O_RDONLY);
    if (fd < 0) {
        ALOGE("Failed to open %s\n", RTC_PATH);
        return ::ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_SERVICE_SPECIFIC));
    }

    struct rtc_time rtc_tm;
    int retval = ioctl(fd, RTC_RD_TIME, &rtc_tm);
    close (fd);
    if (retval == -1) {
        ALOGE("Failed RTC_RD_TIME ioctl");
        return ::ndk::ScopedAStatus(AStatus_fromExceptionCode(EX_SERVICE_SPECIFIC));
    }

	struct tm time;
	time.tm_year = rtc_tm.tm_year;
	time.tm_mon = rtc_tm.tm_mon;
	time.tm_mday = rtc_tm.tm_mday;
	time.tm_hour = rtc_tm.tm_hour;
	time.tm_min = rtc_tm.tm_min;
	time.tm_sec = rtc_tm.tm_sec;

#ifdef __aarch64__
    time_t now = mktime(&time);
#else
    time64_t now = mktime64(((unsigned int)rtc_tm.tm_year + 1900), rtc_tm.tm_mon + 1,
		   rtc_tm.tm_mday, rtc_tm.tm_hour, rtc_tm.tm_min, rtc_tm.tm_sec);
#endif

	char buf[20];
#ifdef __aarch64__
	retval = sprintf(buf, "%ld\n", now);
#else
	retval = sprintf(buf, "%lld\n", now);
#endif
	*_aidl_return = buf;

    return ::ndk::ScopedAStatus::ok();
}

::ndk::ScopedAStatus Rtc::setWakeupAlarm(int64_t triggerAtMillis) {
    if (write_sys(WAKEALARM_PATH, triggerAtMillis) < 0) {
        return ::ndk::ScopedAStatus::fromExceptionCodeWithMessage(EX_ILLEGAL_ARGUMENT,
                "Failed to set wakeup alarm");
    }
    return ::ndk::ScopedAStatus::ok();
}

}  // namespace rtc
}  // namespace hardware
}  // namespace hardkernel
}  // namespace vendor
}  // namespace aidl
