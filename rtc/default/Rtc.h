/*
 * Copyright (C) 2025 The Android Open Source Project
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

#pragma once

#include <aidl/vendor/hardkernel/hardware/rtc/BnRtc.h>
#define LOG_TAG "vendor.hardkernel.hardware.rtc-service"

namespace aidl {
namespace vendor {
namespace hardkernel {
namespace hardware {
namespace rtc {

class Rtc : public BnRtc {
    ::ndk::ScopedAStatus getTime(std::string *_aidl_return) override;
    ::ndk::ScopedAStatus setWakeupAlarm(int64_t triggerAtMillis) override;
};

}  // namespace rtc 
}  // namespace hardware
}  // namespace hardkernel
}  // namespace vendor
}  // namespace aidl
