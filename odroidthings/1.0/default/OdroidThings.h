/*
 *    Copyright (c) 2019 Sangchul Go <luke.go@hardkernel.com>
 *
 *    OdroidThings is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    OdroidThings is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with OdroidThings.
 *    If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef VENDOR_HARDKERNEL_HARDWARE_ODROIDTHINGS_V1_0_H
#define VENDOR_HARDKERNEL_HARDWARE_ODROIDTHINGS_V1_0_H

#include <vendor/hardkernel/hardware/odroidthings/1.0/IOdroidThings.h>

#include <hardware/odroidThings.h>
#include <map>

#define INIT_CALLBACKFUNC(__cbList, n) \
    static void __callback##n() { \
        __cbList[(n)]->doCallback(); \
    }
#define CALLBACKFUNC(n)  __callback##n
#define CASE_CALLBACK(n) case (n): cb = &(CALLBACKFUNC(n)); break

namespace vendor {
namespace hardkernel {
namespace hardware {
namespace odroidthings {
namespace V1_0 {
namespace implementation {

using ::vendor::hardkernel::hardware::odroidthings::V1_0::IOdroidThings;
using gpio_callback = ::vendor::hardkernel::hardware::odroidthings::V1_0::IOdroidThingsGpioCallback;

using ::hardware::hardkernel::odroidthings::things_device_t;
using ::hardware::hardkernel::odroidthings::things_module_t;
using ::hardware::hardkernel::odroidthings::function_t;

using ::android::hardware::hidl_vec;
using ::android::hardware::hidl_string;
using ::android::hardware::Return;
using ::android::hardware::Void;
using ::android::sp;

struct OdroidThings : public IOdroidThings {
public:
    OdroidThings();
    ~OdroidThings();

    // Method to wrap legacy HAL with OdroidThings class
    static IOdroidThings* getInstance();

    // V1_0::IOdroidThings methods
    Return<void> getPinNameList(getPinNameList_cb _hidl_cb) override;
    Return<void> getListOf(int8_t mode, getListOf_cb _hidl_cb) override;

    // gpio
    Return<void> setDirection(int32_t pin, Direction direction) override;
    Return<void> gpio_setValue(int32_t pin, bool value) override;
    Return<bool> gpio_getValue(int32_t pin) override;
    Return<void> gpio_setActiveType(int32_t pin, int32_t activeType) override;
    Return<void> gpio_setEdgeTriggerType(int32_t pin, int32_t edgeTriggerType) override;
    Return<void> gpio_registerCallback(int32_t pin,
            const sp<gpio_callback>& gpioCallback) override;
    Return<void> gpio_unregisterCallback(int32_t pin) override;

    // pwm
    Return<void> pwm_open(int32_t pin) override;
    Return<void> pwm_close(int32_t pin) override;
    Return<bool> pwm_setEnable(int32_t pin, bool enabled) override;
    Return<bool> pwm_setDutyCycle(int32_t pin, double cycle_rate) override;
    Return<bool> pwm_setFrequency(int32_t pin, double frequency_hz) override;

    // i2c
    Return<void> i2c_open(int32_t nameIdx, uint32_t address, int32_t idx) override;
    Return<void> i2c_close(int32_t idx) override;
    Return<void> i2c_readRegBuffer(int32_t idx, uint32_t reg, int32_t length, i2c_readRegBuffer_cb _hidl_cb) override;
    Return<Result> i2c_writeRegBuffer(int32_t idx, uint32_t reg, const hidl_vec<uint8_t>& buffer, int32_t length) override;
private:
    static things_device_t* openHal();
    things_device_t *mDevice;
    static OdroidThings* sInstance;
    static std::map<int, sp<gpio_callback>> callbackList;

    // call back functions for gpio pins
    INIT_CALLBACKFUNC(callbackList, 0);
    INIT_CALLBACKFUNC(callbackList, 1);
    INIT_CALLBACKFUNC(callbackList, 2);
    INIT_CALLBACKFUNC(callbackList, 3);
    INIT_CALLBACKFUNC(callbackList, 4);
    INIT_CALLBACKFUNC(callbackList, 5);
    INIT_CALLBACKFUNC(callbackList, 6);
    INIT_CALLBACKFUNC(callbackList, 7);
    INIT_CALLBACKFUNC(callbackList, 8);
    INIT_CALLBACKFUNC(callbackList, 9);
    INIT_CALLBACKFUNC(callbackList, 10);
    INIT_CALLBACKFUNC(callbackList, 11);
    INIT_CALLBACKFUNC(callbackList, 12);
    INIT_CALLBACKFUNC(callbackList, 13);
    INIT_CALLBACKFUNC(callbackList, 14);
    INIT_CALLBACKFUNC(callbackList, 15);
    INIT_CALLBACKFUNC(callbackList, 16);
    INIT_CALLBACKFUNC(callbackList, 17);
    INIT_CALLBACKFUNC(callbackList, 18);
    INIT_CALLBACKFUNC(callbackList, 19);
    INIT_CALLBACKFUNC(callbackList, 20);
    INIT_CALLBACKFUNC(callbackList, 21);
    INIT_CALLBACKFUNC(callbackList, 22);
    INIT_CALLBACKFUNC(callbackList, 23);
    INIT_CALLBACKFUNC(callbackList, 24);
    INIT_CALLBACKFUNC(callbackList, 25);
    INIT_CALLBACKFUNC(callbackList, 26);
    INIT_CALLBACKFUNC(callbackList, 27);
    INIT_CALLBACKFUNC(callbackList, 28);
    INIT_CALLBACKFUNC(callbackList, 29);
    INIT_CALLBACKFUNC(callbackList, 30);
    INIT_CALLBACKFUNC(callbackList, 31);
    INIT_CALLBACKFUNC(callbackList, 32);
    INIT_CALLBACKFUNC(callbackList, 33);
    INIT_CALLBACKFUNC(callbackList, 34);
    INIT_CALLBACKFUNC(callbackList, 35);
    INIT_CALLBACKFUNC(callbackList, 36);
    INIT_CALLBACKFUNC(callbackList, 37);
    INIT_CALLBACKFUNC(callbackList, 38);
    INIT_CALLBACKFUNC(callbackList, 39);
};

} // namespace implementation
} // namespace V1_0
} // namespace odroidthings
} // namespace hardware
} // namespace hardkernel
} // namespace vendor
#endif // VENDOR_HARDKERNEL_HARDWARE_ODROIDTHINGS_V1_0_H
