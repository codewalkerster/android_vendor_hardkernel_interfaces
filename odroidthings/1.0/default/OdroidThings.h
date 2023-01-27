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

#include <ctime>
#include <log/log.h>
#include <map>
#include <thread>

#define CALLBACK_OFFSET_UART    40
// 30ms
#define CALLBACK_IGNORE_DELTA    10000000

#define INIT_GPIO_CALLBACKFUNC(__cbList,  n) \
    static struct timespec __lastTick##n; \
    static void __callback_##n() { \
        struct timespec __currentTick; \
        clock_gettime(CLOCK_MONOTONIC, &__currentTick); \
        auto delta = ((__currentTick.tv_sec - __lastTick##n.tv_sec) * 1000000000) \
        + (__currentTick.tv_nsec - __lastTick##n.tv_nsec); \
        if (delta > tickDelta) { \
            __lastTick##n.tv_sec = __currentTick.tv_sec; \
            __lastTick##n.tv_nsec = __currentTick.tv_nsec; \
            __cbList[(n)]->doCallback(); \
        } \
    }

#define INIT_UART_CALLBACKFUNC(__cbList,  n) \
    static void __callback_##n() { \
            __cbList[(n)]->doCallback(); \
    }

#define INIT_CALLBACK_TICK(n) \
    struct timespec OdroidThings::__lastTick##n

#define CALLBACKFUNC(n)  __callback_##n

#define CASE_GPIO_CALLBACK(n) \
    case (n): { \
                  clock_gettime(CLOCK_MONOTONIC, &__lastTick##n); \
                  cb = &(CALLBACKFUNC(n)); \
              } \
              break

#define CASE_UART_CALLBACK(n) \
    case (n): { \
                  cb = &(CALLBACKFUNC(n)); \
              } \
              break

namespace vendor::hardkernel::hardware::odroidthings::V1_0::implementation {

using ::vendor::hardkernel::hardware::odroidthings::V1_0::IOdroidThings;
using Callback = ::vendor::hardkernel::hardware::odroidthings::V1_0::IOdroidThingsCallback;

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
    Return<void> getListOf(uint8_t mode, getListOf_cb _hidl_cb) override;

    // gpio
    Return<void> setDirection(int32_t pin, Direction direction) override;
    Return<void> gpio_setValue(int32_t pin, bool value) override;
    Return<bool> gpio_getValue(int32_t pin) override;
    Return<void> gpio_setActiveType(int32_t pin, int32_t activeType) override;
    Return<void> gpio_setEdgeTriggerType(int32_t pin, int32_t edgeTriggerType) override;
    Return<void> gpio_registerCallback(int32_t pin,
            const sp<Callback>& gpioCallback) override;
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
    Return<void> i2c_read(int32_t idx, int32_t length, i2c_read_cb _hidl_cb) override;
    Return<void> i2c_readRegBuffer(int32_t idx, uint32_t reg, int32_t length, i2c_readRegBuffer_cb _hidl_cb) override;
    Return<void> i2c_readRegWord(int32_t idx, uint32_t reg, i2c_readRegWord_cb _hidl_cb) override;
    Return<void> i2c_readRegByte(int32_t idx, uint32_t reg, i2c_readRegByte_cb _hidl_cb) override;
    Return<Result> i2c_write(int32_t idx, const hidl_vec<uint8_t>& buffer, int32_t length) override;
    Return<Result> i2c_writeRegBuffer(int32_t idx, uint32_t reg, const hidl_vec<uint8_t>& buffer, int32_t length) override;
    Return<Result> i2c_writeRegWord(int32_t idx, uint32_t reg, const uint16_t buffer) override;
    Return<Result> i2c_writeRegByte(int32_t idx, uint32_t reg, const uint8_t buffer) override;

    // uart
    Return<void> uart_open(int32_t idx) override;
    Return<void> uart_close(int32_t idx) override;
    Return<bool> uart_flush(int32_t idx, int32_t direction) override;
    Return<bool> uart_sendBreak(int32_t idx, int32_t duration) override;
    Return<bool> uart_setBaudrate(int32_t idx, int32_t rate) override;
    Return<bool> uart_setDataSize(int32_t idx, int32_t size) override;
    Return<bool> uart_setHardwareFlowControl(int32_t idx, int32_t mode) override;
    Return<bool> uart_setParity(int32_t idx, int32_t mode) override;
    Return<bool> uart_setStopBits(int32_t idx, int32_t bits) override;
    Return<void> uart_read(int32_t idx, int32_t length, uart_read_cb _hidl_cb) override;
    Return<int32_t> uart_write(int32_t idx, const hidl_vec<uint8_t>& buffer, int32_t length) override;
    Return<void> uart_registerCallback(int32_t idx,
            const sp<Callback>& uartCallback) override;
    Return<void> uart_unregisterCallback(int32_t idx) override;

    // spi
    Return<void> spi_open(int32_t idx) override;
    Return<void> spi_close(int32_t idx) override;
    Return<bool> spi_setBitJustification(int32_t idx, uint8_t justification) override;
    Return<bool> spi_setBitsPerWord(int32_t idx, uint8_t bits) override;
    Return<bool> spi_setMode(int32_t idx, uint8_t mode) override;
    Return<bool> spi_setCsChange(int32_t idx, bool cs) override;
    Return<bool> spi_setDelay(int32_t idx, uint16_t delay) override;
    Return<bool> spi_setFrequency(int32_t idx, uint32_t frequency) override;
    Return<void> spi_transfer(int32_t idx, const hidl_vec<uint8_t>& txBuffer, int32_t length, spi_transfer_cb _hidl_cb) override;
    Return<void> spi_read(int32_t idx, int32_t length, spi_read_cb _hidl_cb) override;
    Return<bool> spi_write(int32_t idx, const hidl_vec<uint8_t>& txBuffer, int32_t length) override;

private:
    static things_device_t* openHal();
    things_device_t *mDevice;
    static OdroidThings* sInstance;
    static std::map<int, sp<Callback>> callbackList;
    static long tickDelta;

    // call back functions for gpio pins
    INIT_GPIO_CALLBACKFUNC(callbackList, 3);
    INIT_GPIO_CALLBACKFUNC(callbackList, 5);
    INIT_GPIO_CALLBACKFUNC(callbackList, 7);
    INIT_GPIO_CALLBACKFUNC(callbackList, 8);
    INIT_GPIO_CALLBACKFUNC(callbackList, 10);
    INIT_GPIO_CALLBACKFUNC(callbackList, 11);
    INIT_GPIO_CALLBACKFUNC(callbackList, 12);
    INIT_GPIO_CALLBACKFUNC(callbackList, 13);
    INIT_GPIO_CALLBACKFUNC(callbackList, 15);
    INIT_GPIO_CALLBACKFUNC(callbackList, 16);
    INIT_GPIO_CALLBACKFUNC(callbackList, 18);
    INIT_GPIO_CALLBACKFUNC(callbackList, 19);
    INIT_GPIO_CALLBACKFUNC(callbackList, 21);
    INIT_GPIO_CALLBACKFUNC(callbackList, 22);
    INIT_GPIO_CALLBACKFUNC(callbackList, 23);
    INIT_GPIO_CALLBACKFUNC(callbackList, 24);
    INIT_GPIO_CALLBACKFUNC(callbackList, 26);
    INIT_GPIO_CALLBACKFUNC(callbackList, 27);
    INIT_GPIO_CALLBACKFUNC(callbackList, 28);
    INIT_GPIO_CALLBACKFUNC(callbackList, 29);
    INIT_GPIO_CALLBACKFUNC(callbackList, 31);
    INIT_GPIO_CALLBACKFUNC(callbackList, 32);
    INIT_GPIO_CALLBACKFUNC(callbackList, 33);
    INIT_GPIO_CALLBACKFUNC(callbackList, 35);
    INIT_GPIO_CALLBACKFUNC(callbackList, 36);

    // call back functions for Uart pins
    INIT_UART_CALLBACKFUNC(callbackList, 40);
    INIT_UART_CALLBACKFUNC(callbackList, 41);
    INIT_UART_CALLBACKFUNC(callbackList, 42);
    INIT_UART_CALLBACKFUNC(callbackList, 43);
};

} // namespace vendor::hardkernel::hardware::odroidthings::V1_0::implementation
#endif // VENDOR_HARDKERNEL_HARDWARE_ODROIDTHINGS_V1_0_H
